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
    # Tracker modern, biasanya ada di modul utama
    "csrt": cv2.TrackerCSRT_create,
    "kcf": cv2.TrackerKCF_create,

    # Tracker yang lebih tua, sekarang berada di modul 'legacy'
    "boosting": cv2.legacy.TrackerBoosting_create,
    "mil": cv2.legacy.TrackerMIL_create,
    "tld": cv2.legacy.TrackerTLD_create,
    "medianflow": cv2.legacy.TrackerMedianFlow_create,
    "mosse": cv2.legacy.TrackerMOSSE_create
}


# ==============================================================================
# KELAS KONTROLER 
# ==============================================================================
class PIDController:
    def __init__(self, kp, ki, kd, max_integral):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_integral = max_integral
        self.integral = 0
        self.prev_error = 0
    def calculate(self, error):
        self.integral += error
        if self.integral > self.max_integral: self.integral = self.max_integral
        elif self.integral < -self.max_integral: self.integral = -self.max_integral
        derivative = error - self.prev_error
        adjustment = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error
        return adjustment
    def reset(self):
        self.integral = 0
        self.prev_error = 0

class ServoController:
    def __init__(self, config):
        if not PCA9685_AVAILABLE: self.pwm = None; return
        self.config = config["servo"]
        try:
            self.pwm = PCA9685(0x40)
            self.pwm.setPWMFreq(50)
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
# KONFIGURASI UTAMA 
# ==============================================================================

CONFIG = {
    "camera": {"width": 640, "height": 480},
    "servo": {
        "pan_channel": 0,
        "tilt_channel": 1,
        "pan_center_pulse": 1500,
        "tilt_center_pulse": 1000,
        "pan_min_pulse": 750,
        "pan_max_pulse": 2250,
        "tilt_min_pulse": 800,
        "tilt_max_pulse": 2200,
        "pan_inversion": -1,
        "tilt_inversion": 1
    },
    "pid_pan":  {"kp": 0.15, "ki": 0.001, "kd": 0.05, "max_integral": 300},
    "pid_tilt": {"kp": 0.2, "ki": 0.001, "kd": 0.06, "max_integral": 300},
    #pengaturan tracking
    "tracking": {
        # Ubah nilai di bawah ini untuk mengganti algoritma tracker
        # Pilihan: "csrt", "kcf", "tld", "mil", "boosting", "medianflow", "mosse"
        "tracker_type": "mosse", 
        "tolerance_ratio": 0.05
    }
}

# ==============================================================================
# LOGIKA UTAMA APLIKASI
# ==============================================================================

# [PERUBAHAN 3]: Fungsi helper untuk membuat tracker berdasarkan CONFIG
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

tracker = None
tracking = False
bbox = None

print(f"INFO: Tracker diatur ke {CONFIG['tracking']['tracker_type'].upper()}.")
print("INFO: Tekan 's' untuk memilih objek, lalu tekan 'ENTER' atau 'SPACE' untuk memulai tracking.")
print("INFO: Tekan 'q' untuk keluar dari program secara normal.")

while True:
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) 
    frame = cv2.flip(frame, 1)

    frame_height, frame_width, _ = frame.shape
    frame_center_x = frame_width // 2
    frame_center_y = frame_height // 2

    if tracking and tracker is not None:
        success, bbox = tracker.update(frame)

        if success:
            (x, y, w, h) = [int(v) for v in bbox]
            bbox_center_x = x + w // 2
            bbox_center_y = y + h // 2
            
            error_x = bbox_center_x - frame_center_x
            error_y = bbox_center_y - frame_center_y 
            
            tolerance_x = CONFIG["tracking"]["tolerance_ratio"] * frame_width
            tolerance_y = CONFIG["tracking"]["tolerance_ratio"] * frame_height

            if abs(error_x) > tolerance_x or abs(error_y) > tolerance_y:
                adjustment_x = pid_pan.calculate(error_x)
                adjustment_y = pid_tilt.calculate(error_y)
                
                current_pan -= CONFIG["servo"]["pan_inversion"] * adjustment_x
                current_tilt -= CONFIG["servo"]["tilt_inversion"] * adjustment_y
                
                servo_controller.update(current_pan, current_tilt)
            else:
                pid_pan.reset()
                pid_tilt.reset()

            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        else:
            cv2.putText(frame, "Tracking Gagal", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            tracking = False
            tracker = None

    cv2.imshow(f"Object Tracking - {CONFIG['tracking']['tracker_type'].upper()} with Servo Control", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('s') and not tracking:
        selection_frame = picam2.capture_array()
        selection_frame = cv2.cvtColor(selection_frame, cv2.COLOR_BGR2RGB)
        selection_frame = cv2.flip(selection_frame, 1)

        bbox = cv2.selectROI("Select Object", selection_frame, fromCenter=False, showCrosshair=True)
        cv2.destroyWindow("Select Object")
        
        if bbox[2] > 0 and bbox[3] > 0:
            # Membuat instance tracker secara dinamis
            tracker = create_tracker()
            tracker.init(selection_frame, bbox)
            tracking = True
            pid_pan.reset()
            pid_tilt.reset()
        else:
            print("PERINGATAN: Seleksi tidak valid, objek tidak dipilih.")
    
    elif key == ord('q'):
        break

print("INFO: Menutup program...")
if servo_controller.pwm:
    servo_controller.set_initial_position()
picam2.stop()
cv2.destroyAllWindows()