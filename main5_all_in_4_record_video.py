import sys
import cv2
from picamera2 import Picamera2
import time

# --- PENGATURAN LOKASI LIBRARY SERVO ---
try:
    sys.path.append('/home/pi/project/select_object_tracking/resources/servo')
    from PCA9685 import PCA9685
    PCA9685_AVAILABLE = True
    print("INFO: Pustaka PCA9685 berhasil diimpor.")
except ImportError:
    print("PERINGATAN: Pustaka PCA9685 tidak ditemukan. Kontrol servo akan dinonaktifkan.")
    PCA9685_AVAILABLE = False

# dictionary untuk memilih tracker
TRACKER_BUILDERS = {
    "csrt": cv2.TrackerCSRT_create, "kcf": cv2.TrackerKCF_create,
    "boosting": cv2.legacy.TrackerBoosting_create, "mil": cv2.legacy.TrackerMIL_create,
    "tld": cv2.legacy.TrackerTLD_create, "medianflow": cv2.legacy.TrackerMedianFlow_create,
    "mosse": cv2.legacy.TrackerMOSSE_create
}

# ==============================================================================
# KELAS KONTROLER 
# ==============================================================================
class PIDController:
    def __init__(self, kp, ki, kd, max_integral):
        self.kp, self.ki, self.kd = kp, ki, kd; self.max_integral = max_integral
        self.integral = 0; self.prev_error = 0
    def calculate(self, error):
        self.integral += error
        if self.integral > self.max_integral: self.integral = self.max_integral
        elif self.integral < -self.max_integral: self.integral = -self.max_integral
        derivative = error - self.prev_error
        adjustment = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error
        return adjustment
    def reset(self):
        self.integral = 0; self.prev_error = 0

class ServoController:
    def __init__(self, config):
        if not PCA9685_AVAILABLE: self.pwm = None; return
        self.config = config["servo"]
        try:
            self.pwm = PCA9685(0x40); self.pwm.setPWMFreq(50)
            self.set_initial_position()
            print("INFO: Servo controller berhasil diinisialisasi.")
        except Exception as e:
            print(f"ERROR: Gagal menginisialisasi PCA9685. Periksa koneksi I2C. - {e}")
            self.pwm = None
    def set_initial_position(self):
        if self.pwm:
            print("INFO: Mengatur servo ke posisi awal (tengah)...")
            self.pwm.setServoPulse(self.config["pan_channel"], self.config["pan_center_pulse"])
            self.pwm.setServoPulse(self.config["tilt_channel"], self.config["tilt_center_pulse"])
    def update(self, pan_pulse, tilt_pulse):
        if not self.pwm: return
        clamped_pan = max(self.config["pan_min_pulse"], min(self.config["pan_max_pulse"], pan_pulse))
        clamped_tilt = max(self.config["tilt_min_pulse"], min(self.config["tilt_max_pulse"], tilt_pulse))
        try:
            self.pwm.setServoPulse(self.config["pan_channel"], int(clamped_pan))
            self.pwm.setServoPulse(self.config["tilt_channel"], int(clamped_tilt))
        except Exception as e:
            print(f"ERROR: Gagal menggerakkan servo: {e}")

# ==============================================================================
# KONFIGURASI UTAMA (FOKUS KALIBRASI DI SINI)
# ==============================================================================
CONFIG = {
    "camera": {"width": 640, "height": 480},
    "servo": {
        "pan_channel": 0, "tilt_channel": 1, "pan_center_pulse": 1500, "tilt_center_pulse": 1000,
        "pan_min_pulse": 750, "pan_max_pulse": 2250, "tilt_min_pulse": 800, "tilt_max_pulse": 2200,
        "pan_inversion": -1, "tilt_inversion": 1, "manual_step_pulse": 20
    },
    "pid_pan":  {"kp": 0.15, "ki": 0.001, "kd": 0.05, "max_integral": 300},
    "pid_tilt": {"kp": 0.2, "ki": 0.001, "kd": 0.06, "max_integral": 300},
    "tracking": {"tracker_type": "csrt", "tolerance_ratio": 0.05}
}

# ==============================================================================
# LOGIKA UTAMA APLIKASI
# ==============================================================================
def create_tracker():
    tracker_name = CONFIG["tracking"]["tracker_type"].lower()
    builder = TRACKER_BUILDERS.get(tracker_name)
    if builder:
        print(f"INFO: Menggunakan tracker {tracker_name.upper()}.")
        return builder()
    else:
        print(f"PERINGATAN: Tracker '{tracker_name}' tidak ditemukan. Menggunakan CSRT sebagai default.")
        return TRACKER_BUILDERS["csrt"]()

servo_controller = ServoController(CONFIG)
pid_pan = PIDController(**CONFIG["pid_pan"])
pid_tilt = PIDController(**CONFIG["pid_tilt"])
current_pan = float(CONFIG["servo"]["pan_center_pulse"])
current_tilt = float(CONFIG["servo"]["tilt_center_pulse"])

picam2 = Picamera2()
cam_config = picam2.create_preview_configuration(main={"size": (CONFIG["camera"]["width"], CONFIG["camera"]["height"])})
picam2.configure(cam_config)
picam2.start()
time.sleep(1.0)

# Variabel untuk Kontrol Zoom
full_resolution = picam2.camera_properties['PixelArraySize']
current_zoom_rect = [0, 0, full_resolution[0], full_resolution[1]]
zoom_step = 0.1
print(f"INFO: Resolusi sensor penuh: {full_resolution}")
print("INFO: Tekan '+' untuk Zoom In, '-' untuk Zoom Out.")
print("INFO: Gunakan tombol panah untuk kontrol manual servo.")

# --- TAMBAHAN UNTUK REKAM VIDEO ---
is_recording = False
video_writer = None
print("INFO: Tekan 'r' untuk memulai/menghentikan perekaman video.")
# ------------------------------------

tracker = None; tracking = False; bbox = None
print(f"INFO: Tracker diatur ke {CONFIG['tracking']['tracker_type'].upper()}.")
print("INFO: Tekan 's' untuk memilih objek, lalu tekan 'ENTER' atau 'SPACE' untuk memulai tracking.")
print("INFO: Tekan 'q' untuk keluar dari program secara normal.")

while True:
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame = cv2.flip(frame, 1)

    if tracking and tracker is not None:
        success, bbox = tracker.update(frame)
        if success:
            (x, y, w, h) = [int(v) for v in bbox]
            frame_height, frame_width, _ = frame.shape
            bbox_center_x = x + w // 2; bbox_center_y = y + h // 2
            error_x = bbox_center_x - (frame_width // 2)
            error_y = bbox_center_y - (frame_height // 2)
            tolerance_x = CONFIG["tracking"]["tolerance_ratio"] * frame_width
            tolerance_y = CONFIG["tracking"]["tolerance_ratio"] * frame_height
            if abs(error_x) > tolerance_x or abs(error_y) > tolerance_y:
                adjustment_x = pid_pan.calculate(error_x)
                adjustment_y = pid_tilt.calculate(error_y)
                current_pan -= CONFIG["servo"]["pan_inversion"] * adjustment_x
                current_tilt -= CONFIG["servo"]["tilt_inversion"] * adjustment_y
                servo_controller.update(current_pan, current_tilt)
            else:
                pid_pan.reset(); pid_tilt.reset()
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        else:
            cv2.putText(frame, "Tracking Gagal", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            tracking = False; tracker = None

    # --- TAMBAHAN: INDIKATOR & PROSES REKAM ---
    if is_recording:
        cv2.circle(frame, (30, 30), 10, (0, 0, 255), -1)
        cv2.putText(frame, "REC", (50, 37), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        if video_writer:
            bgr_frame_for_writing = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            video_writer.write(bgr_frame_for_writing)

    cv2.imshow(f"Object Tracking - {CONFIG['tracking']['tracker_type'].upper()} with Servo Control", frame)
    key = cv2.waitKey(1)

    if key == ord('s') and not tracking:
        selection_frame = cv2.cvtColor(picam2.capture_array(), cv2.COLOR_BGR2RGB)
        selection_frame = cv2.flip(selection_frame, 1)
        bbox = cv2.selectROI("Select Object", selection_frame, fromCenter=False, showCrosshair=True)
        cv2.destroyWindow("Select Object")
        if bbox[2] > 0 and bbox[3] > 0:
            tracker = create_tracker(); tracker.init(selection_frame, bbox)
            tracking = True; pid_pan.reset(); pid_tilt.reset()
        else:
            print("PERINGATAN: Seleksi tidak valid, objek tidak dipilih.")
    elif key == ord('+') or key == ord('='):
        new_w = int(current_zoom_rect[2] * (1 - zoom_step))
        new_h = int(current_zoom_rect[3] * (1 - zoom_step))
        if new_w > 64 and new_h > 64:
            new_x = current_zoom_rect[0] + (current_zoom_rect[2] - new_w) // 2
            new_y = current_zoom_rect[1] + (current_zoom_rect[3] - new_h) // 2
            current_zoom_rect = [new_x, new_y, new_w, new_h]
            picam2.set_controls({"ScalerCrop": current_zoom_rect})
    elif key == ord('-'):
        new_w = int(current_zoom_rect[2] * (1 + zoom_step))
        new_h = int(current_zoom_rect[3] * (1 + zoom_step))
        if new_w <= full_resolution[0] and new_h <= full_resolution[1]:
            new_x = current_zoom_rect[0] - (new_w - current_zoom_rect[2]) // 2
            new_y = current_zoom_rect[1] - (new_h - current_zoom_rect[3]) // 2
            current_zoom_rect = [max(0, new_x), max(0, new_y), new_w, new_h]
            picam2.set_controls({"ScalerCrop": current_zoom_rect})
    elif key in [81, 83, 82, 84]:
        if tracking:
            print("INFO: Kontrol manual diaktifkan, tracking dihentikan.")
            tracking = False; tracker = None; pid_pan.reset(); pid_tilt.reset()
        step = CONFIG["servo"]["manual_step_pulse"]
        if key == 81: current_pan -= step
        elif key == 83: current_pan += step
        elif key == 82: current_tilt += step
        elif key == 84: current_tilt -= step
        servo_controller.update(current_pan, current_tilt)
    # --- TAMBAHAN: LOGIKA UNTUK MEREKAM VIDEO ---
    elif key == ord('r'):
        if not is_recording:
            is_recording = True
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            video_filename = f"recording_{timestamp}.mp4"
            frame_height, frame_width, _ = frame.shape
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            video_writer = cv2.VideoWriter(video_filename, fourcc, 20.0, (frame_width, frame_height))
            print(f"INFO: Mulai merekam video ke file: {video_filename}")
        else:
            is_recording = False
            if video_writer:
                video_writer.release()
                video_writer = None
            print("INFO: Perekaman video dihentikan.")
    elif key == ord('q'):
        break

print("INFO: Menutup program...")
# --- TAMBAHAN: CLEANUP VIDEO WRITER ---
if is_recording and video_writer:
    print("INFO: Menyimpan sisa rekaman video...")
    video_writer.release()
# ------------------------------------
if servo_controller.pwm:
    servo_controller.set_initial_position()
picam2.stop()
cv2.destroyAllWindows()