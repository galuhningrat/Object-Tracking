#!/usr/bin/env python3
"""
Program Pelacakan Objek Real-Time - VERSI DIPERBAIKI (ANTI-DRIFT FIX)

Perbaikan:
1. Menyetel KI (Integral Gain) ke 0 untuk menghilangkan drift/gerak sendiri.
2. Mengatur max_integral ke 0 untuk menonaktifkan Anti-Windup pada komponen Integral yang dinonaktifkan.
3. Kp sedikit dinaikkan untuk mengkompensasi Ki=0.
"""

import sys
import os
import cv2
import time
from picamera2 import Picamera2
import numpy as np
from collections import deque

# Setup library servo (tetap)
sys.path.append(os.path.join(os.path.dirname(__file__), 'resources', 'servo'))

try:
    from PCA9685 import PCA9685
    PCA9685_AVAILABLE = True
    print("✓ Driver Servo PCA9685 berhasil diimpor.")
except ImportError:
    print("⚠ PERINGATAN: Pustaka PCA9685 tidak ditemukan. Kontrol servo dinonaktifkan.")
    PCA9685_AVAILABLE = False

# Dictionary tracker (tetap)
TRACKER_BUILDERS = {
    "csrt": cv2.TrackerCSRT_create,
    "kcf": cv2.TrackerKCF_create,
    "mosse": cv2.legacy.TrackerMOSSE_create,
    "mil": cv2.legacy.TrackerMIL_create,
}

# ==============================================================================
# KELAS KONTROLER (Tidak ada perubahan di sini)
# ==============================================================================

class MovingAverageFilter:
    """Filter untuk smoothing nilai servo (mengurangi jitter)."""
    def __init__(self, window_size=3):
        self.window_size = window_size
        self.buffer = deque(maxlen=window_size)
    
    def update(self, value):
        self.buffer.append(value)
        return sum(self.buffer) / len(self.buffer)
    
    def reset(self):
        self.buffer.clear()


class PIDController:
    """Kontroler PID dengan Anti-Windup dan Dead Zone."""
    def __init__(self, kp, ki, kd, max_integral, dead_zone=0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_integral = max_integral
        self.dead_zone = dead_zone  
        self.integral = 0
        self.prev_error = 0
        self.last_time = time.time()
        
    def calculate(self, error):
        # Dead zone - jangan respond jika error terlalu kecil
        if abs(error) < self.dead_zone:
            # PENTING: Jika di dalam Dead Zone, integral tidak boleh diakumulasi,
            # dan output PID harus 0.
            return 0
        
        current_time = time.time()
        delta_time = current_time - self.last_time
        self.last_time = current_time

        if delta_time == 0 or delta_time > 1.0: 
            delta_time = 1/30
        
        # Integral dengan anti-windup (akan dinonaktifkan jika ki=0 dan max_integral=0)
        self.integral += error * delta_time
        self.integral = max(-self.max_integral, min(self.max_integral, self.integral))

        # Derivative
        derivative = (error - self.prev_error) / delta_time
        
        # Output PID
        # Jika self.ki = 0, maka komponen ini hilang.
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        self.prev_error = error
        return output
        
    def reset(self):
        self.integral = 0
        self.prev_error = 0
        self.last_time = time.time()


class ServoController:
    """Kontroler Servo dengan Smoothing dan Rate Limiting."""
    # (Tidak ada perubahan)
    def __init__(self, servo_config):
        self.config = servo_config
        self.current_pan_pulse = float(self.config["pan_center_pulse"])
        self.current_tilt_pulse = float(self.config["tilt_center_pulse"])
        
        # NEW: Smoothing filters
        self.pan_filter = MovingAverageFilter(window_size=3)
        self.tilt_filter = MovingAverageFilter(window_size=3)
        
        # NEW: Rate limiting
        self.last_update_time = time.time()
        self.min_update_interval = 0.05  # Max 20 updates/second

        if not PCA9685_AVAILABLE: 
            self.pwm = None
            return
        
        try:
            self.pwm = PCA9685(0x40) 
            self.pwm.setPWMFreq(50)
            print("✓ Servo controller berhasil diinisialisasi.")
            self.set_initial_position()
        except Exception as e:
            print(f"✗ ERROR: Gagal menginisialisasi PCA9685: {e}")
            print("  Periksa: 1) I2C enabled, 2) HAT terpasang, 3) External power 5V/3A")
            self.pwm = None
            
    def set_initial_position(self):
        if self.pwm:
            pan_center = self.config["pan_center_pulse"]
            tilt_center = self.config["tilt_center_pulse"]
            print(f"→ Mengatur servo ke posisi awal (Pan: {pan_center}, Tilt: {tilt_center})")
            self._send_pulse(pan_center, tilt_center, force=True)
            time.sleep(0.5)

    def _send_pulse(self, pan_pulse, tilt_pulse, force=False):
        """Send pulse dengan rate limiting dan smoothing."""
        # Rate limiting (kecuali force=True)
        if not force:
            current_time = time.time()
            if current_time - self.last_update_time < self.min_update_interval:
                return
            self.last_update_time = current_time
        
        # Apply smoothing
        smoothed_pan = self.pan_filter.update(pan_pulse)
        smoothed_tilt = self.tilt_filter.update(tilt_pulse)
        
        # Clamp
        clamped_pan = max(self.config["pan_min_pulse"], 
                          min(self.config["pan_max_pulse"], smoothed_pan))
        clamped_tilt = max(self.config["tilt_min_pulse"], 
                           min(self.config["tilt_max_pulse"], smoothed_tilt))
        
        self.current_pan_pulse = clamped_pan
        self.current_tilt_pulse = clamped_tilt
        
        try:
            self.pwm.setServoPulse(self.config["pan_channel"], int(clamped_pan))
            self.pwm.setServoPulse(self.config["tilt_channel"], int(clamped_tilt))
        except Exception as e:
            print(f"\n✗ ERROR: Gagal menggerakkan servo: {e}")
            
    def update(self, pan_pulse, tilt_pulse):
        if not self.pwm: return
        self._send_pulse(pan_pulse, tilt_pulse)
            
    def cleanup(self):
        if self.pwm:
            print("\n→ Membersihkan sinyal PWM...")
            self.set_initial_position()
            time.sleep(0.3)
            self.pwm.setPWM(self.config["pan_channel"], 0, 0)
            self.pwm.setPWM(self.config["tilt_channel"], 0, 0)
            print("✓ Servo signals disabled.")


# ==============================================================================
# KONFIGURASI - TUNED (FINAL FIX: ANTI-DRIFT) <-- PERUBAHAN UTAMA DI SINI
# ==============================================================================
CONFIG = {
    "camera": {
        "width": 640, 
        "height": 480,
        "flip": False  
    },
    "servo": {
        "pan_channel": 0, 
        "tilt_channel": 1, 
        "pan_center_pulse": 1500, 
        "tilt_center_pulse": 1500,
        "pan_min_pulse": 750, 
        "pan_max_pulse": 2250, 
        "tilt_min_pulse": 800, 
        "tilt_max_pulse": 2200,
        "pan_inversion": 1,
        "tilt_inversion": -1,
        "manual_step_pulse": 30
    },
    # PID TUNED FINAL: Ki=0 untuk menghilangkan drift. Kp/Kd sedikit dinaikkan.
    "pid_pan":  {"kp": 0.05, "ki": 0.0, "kd": 0.02, "max_integral": 0, "dead_zone": 5}, 
    "pid_tilt": {"kp": 0.06, "ki": 0.0, "kd": 0.025, "max_integral": 0, "dead_zone": 5},
    "tracking": {
        "tracker_type": "csrt",
        "tolerance_ratio": 0.08
    }
}


# ==============================================================================
# FUNGSI UTAMA (Tidak ada perubahan substansial)
# ==============================================================================
def run_tracking():
    global current_pan, current_tilt
    
    current_pan = CONFIG["servo"]["pan_center_pulse"]
    current_tilt = CONFIG["servo"]["tilt_center_pulse"]
    
    # Inisialisasi dengan dead_zone
    pid_pan = PIDController(**CONFIG["pid_pan"])
    pid_tilt = PIDController(**CONFIG["pid_tilt"])
    servo_controller = ServoController(servo_config=CONFIG["servo"])

    tracker_type = CONFIG["tracking"]["tracker_type"]
    tracker = None
    tracking = False
    bbox = None
    
    is_recording = False
    video_writer = None
    
    picam2 = None
    zoom_step = 0.1
    current_zoom_rect = None
    full_resolution = None

    try:
        # Inisialisasi Kamera
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(
            main={"size": (CONFIG["camera"]["width"], CONFIG["camera"]["height"])},
            queue=False,
        )
        picam2.configure(config)
        picam2.start()
        time.sleep(1.0)
        
        full_resolution = picam2.camera_properties['PixelArraySize']
        current_zoom_rect = [0, 0, full_resolution[0], full_resolution[1]]
        
        frame_width = CONFIG["camera"]["width"]
        frame_height = CONFIG["camera"]["height"]
        center_x = frame_width // 2
        center_y = frame_height // 2
        
        print("\n" + "="*70)
        print("  OBJECT TRACKING SYSTEM - FINAL TUNING (ANTI-DRIFT FIX)")
        print("="*70)
        print(f"  Camera: {frame_width}x{frame_height} | Sensor: {full_resolution}")
        print(f"  Tracker: {tracker_type.upper()} | PID: Kp_Pan={CONFIG['pid_pan']['kp']} | Ki: {CONFIG['pid_pan']['ki']}")
        print("\n  KONTROL:")
        print("    [S]      : Select object & start tracking")
        print("    [I/J/K/L]: Manual servo (Up/Left/Down/Right)")
        print("    [+/-]    : Zoom in/out")
        print("    [R]      : Toggle recording")
        print("    [Q]      : Quit")
        print("="*70 + "\n")

        # FPS counter
        fps_start_time = time.time()
        fps_frame_count = 0
        fps_display = 0

        while True:
            # Capture frame
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Optional flip (disable jika koordinat tracking bermasalah)
            if CONFIG["camera"]["flip"]:
                frame = cv2.flip(frame, 1)

            # FPS calculation
            fps_frame_count += 1
            if time.time() - fps_start_time > 1.0:
                fps_display = fps_frame_count
                fps_frame_count = 0
                fps_start_time = time.time()

            # --- VISUALISASI ---
            # Crosshair center
            cv2.drawMarker(frame, (center_x, center_y), (0, 0, 255), 
                          cv2.MARKER_CROSS, 20, 2)

            # Zona toleransi (Dead zone)
            tolerance_x = int(CONFIG["tracking"]["tolerance_ratio"] * frame_width / 2)
            tolerance_y = int(CONFIG["tracking"]["tolerance_ratio"] * frame_height / 2)
            cv2.rectangle(frame, 
                         (center_x - tolerance_x, center_y - tolerance_y), 
                         (center_x + tolerance_x, center_y + tolerance_y), 
                         (0, 255, 255), 1)
            
            # --- TRACKING LOGIC ---
            if tracking and tracker is not None:
                ok, new_bbox = tracker.update(frame)
                
                if ok:
                    bbox = new_bbox
                    (x, y, w, h) = [int(v) for v in bbox]
                    
                    object_center_x = x + w // 2
                    object_center_y = y + h // 2
                    
                    # Draw object center
                    cv2.circle(frame, (object_center_x, object_center_y), 5, (255, 0, 0), -1)
                    
                    # Error calculation
                    error_x = object_center_x - center_x
                    error_y = object_center_y - center_y

                    # PID Control (zona toleransi sudah di handle di PID.calculate via dead_zone)
                    adjustment_x = pid_pan.calculate(error_x)
                    adjustment_y = pid_tilt.calculate(error_y)
                    
                    # Update servo (hanya jika ada adjustment)
                    if adjustment_x != 0 or adjustment_y != 0:
                        current_pan += CONFIG["servo"]["pan_inversion"] * adjustment_x
                        current_tilt += CONFIG["servo"]["tilt_inversion"] * adjustment_y
                        servo_controller.update(current_pan, current_tilt)
                    
                    # Visualisasi bounding box
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    
                    # Status overlay
                    status_color = (0, 255, 0) if abs(error_x) < tolerance_x and abs(error_y) < tolerance_y else (0, 165, 255)
                    cv2.putText(frame, f"Tracking: OK | Err: ({error_x:+4.0f}, {error_y:+4.0f})", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)

                else:
                    # Tracking failed
                    cv2.putText(frame, "TRACKING LOST - Press [S] to reselect", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    tracking = False
                    tracker = None
                    pid_pan.reset()
                    pid_tilt.reset()
                    # Reset zoom saat tracking loss
                    current_zoom_rect = [0, 0, full_resolution[0], full_resolution[1]]
                    picam2.set_controls({"ScalerCrop": current_zoom_rect})
            
            else:
                cv2.putText(frame, "Press [S] to Select Object", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            
            # Pulse overlay (bottom right)
            cv2.putText(frame, f"Pan: {int(current_pan):4d} | Tilt: {int(current_tilt):4d} | FPS: {fps_display}", 
                       (10, frame_height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # Recording indicator
            if is_recording:
                cv2.circle(frame, (frame_width - 30, 30), 12, (0, 0, 255), -1)
                cv2.putText(frame, "REC", (frame_width - 65, 37), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                if video_writer and video_writer.isOpened():
                    video_writer.write(frame)
            
            # Display
            cv2.imshow(f"Object Tracking [{tracker_type.upper()}]", frame)
            key = cv2.waitKey(1) & 0xFF

            # --- KEY HANDLING ---
            
            # [S] - Select Object
            if key == ord('s') and not tracking:
                print("\n→ Mode seleksi objek. Pilih area dengan mouse, tekan ENTER/SPACE.")
                temp_frame = picam2.capture_array()
                temp_frame = cv2.cvtColor(temp_frame, cv2.COLOR_RGB2BGR)
                if CONFIG["camera"]["flip"]:
                    temp_frame = cv2.flip(temp_frame, 1)
                
                bbox = cv2.selectROI("Pilih Objek (ENTER untuk konfirmasi)", temp_frame, False, False)
                cv2.destroyWindow("Pilih Objek (ENTER untuk konfirmasi)")
                
                if bbox[2] > 0 and bbox[3] > 0:
                    tracker_builder = TRACKER_BUILDERS.get(tracker_type)
                    if tracker_builder:
                        tracker = tracker_builder()
                        tracker.init(temp_frame, bbox)
                        tracking = True
                        pid_pan.reset()
                        pid_tilt.reset()
                        servo_controller.pan_filter.reset()
                        servo_controller.tilt_filter.reset()
                        print(f"✓ Tracking dimulai dengan {tracker_type.upper()}")
                    else:
                        print(f"✗ Tracker '{tracker_type}' tidak tersedia!")
                else:
                    print("✗ Seleksi dibatalkan atau tidak valid.")
                    
            # [I/J/K/L] - Manual Control (REMAPPED)
            elif key in [ord('i'), ord('j'), ord('k'), ord('l')]:
                if tracking:
                    tracking = False
                    tracker = None
                    pid_pan.reset()
                    pid_tilt.reset()
                    print("\n→ Tracking dihentikan, mode manual aktif.")
                    
                step = CONFIG["servo"]["manual_step_pulse"]
                if key == ord('j'):    # Left
                    current_pan -= step
                    print(f"← Pan Left: {int(current_pan)}", end='\r')
                elif key == ord('l'):  # Right
                    current_pan += step
                    print(f"→ Pan Right: {int(current_pan)}", end='\r')
                elif key == ord('i'):  # Up
                    current_tilt += step
                    print(f"↑ Tilt Up: {int(current_tilt)}", end='\r')
                elif key == ord('k'):  # Down
                    current_tilt -= step
                    print(f"↓ Tilt Down: {int(current_tilt)}", end='\r')

                servo_controller.update(current_pan, current_tilt)
                
            # [+/-] - Zoom
            elif key in [ord('+'), ord('=')]:
                new_w = int(current_zoom_rect[2] * (1 - zoom_step))
                new_h = int(current_zoom_rect[3] * (1 - zoom_step))
                if new_w > 64 and new_h > 64:
                    new_x = current_zoom_rect[0] + (current_zoom_rect[2] - new_w) // 2
                    new_y = current_zoom_rect[1] + (current_zoom_rect[3] - new_h) // 2
                    current_zoom_rect = [new_x, new_y, new_w, new_h]
                    picam2.set_controls({"ScalerCrop": current_zoom_rect})
                    print(f"🔍 Zoom In: {current_zoom_rect}", end='\r')
            
            elif key == ord('-'):
                new_w = int(current_zoom_rect[2] * (1 + zoom_step))
                new_h = int(current_zoom_rect[3] * (1 + zoom_step))
                if new_w <= full_resolution[0] and new_h <= full_resolution[1]:
                    new_x = current_zoom_rect[0] - (new_w - current_zoom_rect[2]) // 2
                    new_y = current_zoom_rect[1] - (new_h - current_zoom_rect[3]) // 2
                    current_zoom_rect = [max(0, new_x), max(0, new_y), new_w, new_h]
                    picam2.set_controls({"ScalerCrop": current_zoom_rect})
                    print(f"🔍 Zoom Out: {current_zoom_rect}", end='\r')

            # [R] - Toggle Recording (FIXED)
            elif key == ord('r'):
                if not is_recording:
                    try:
                        # Ensure recordings folder exists
                        os.makedirs("recordings", exist_ok=True)
                        
                        timestamp = time.strftime("%Y%m%d-%H%M%S")
                        video_filename = f"recordings/recording_{timestamp}.mp4"
                        
                        # Try different codecs in order of preference
                        for codec in ['avc1', 'mp4v', 'XVID']:
                            fourcc = cv2.VideoWriter_fourcc(*codec)
                            video_writer = cv2.VideoWriter(
                                video_filename, fourcc, 20.0, 
                                (frame_width, frame_height)
                            )
                            if video_writer.isOpened():
                                is_recording = True
                                print(f"\n✓ Recording started: {video_filename} (codec: {codec})")
                                break
                        
                        if not is_recording:
                            print("\n✗ ERROR: Tidak bisa membuat video writer. Coba install ffmpeg:")
                            print("   sudo apt install ffmpeg")
                            video_writer = None
                            
                    except Exception as e:
                        print(f"\n✗ ERROR saat memulai recording: {e}")
                        is_recording = False
                        video_writer = None
                else:
                    is_recording = False
                    if video_writer:
                        video_writer.release()
                        video_writer = None
                    print("\n✓ Recording stopped.")
                    
            # [Q] - Quit
            elif key == ord('q'):
                print("\n→ User requested exit...")
                break

    except KeyboardInterrupt:
        print("\n⚠ Program interrupted (Ctrl+C)")
        
    except Exception as e:
        print(f"\n✗ FATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        # Cleanup
        print("\n→ Cleaning up resources...")
        
        if is_recording and video_writer:
            video_writer.release()
            print("  ✓ Video file closed")
            
        servo_controller.cleanup()
        cv2.destroyAllWindows()
        
        if picam2:
            try:
                picam2.stop()
                print("  ✓ Camera stopped")
            except:
                pass
        
        print("\n" + "="*70)
        print("  Program terminated cleanly.")
        print("="*70 + "\n")


if __name__ == "__main__":
    try:
        run_tracking()
    except Exception as e:
        print(f"\n✗ Fatal error at main level: {e}")
        import traceback
        traceback.print_exc()
