import sys
import cv2
from picamera2 import Picamera2
import time

# --- PENGATURAN LOKASI LIBRARY SERVO ---
# Pastikan path ini benar sesuai struktur proyek Anda
try:
    # Ganti path ini jika struktur folder Anda berbeda
    sys.path.append('/home/pi/project/select_object_tracking/resources/servo')
    from PCA9685 import PCA9685
    PCA9685_AVAILABLE = True
    print("INFO: Pustaka PCA9685 berhasil diimpor.")
except ImportError:
    print("PERINGATAN: Pustaka PCA9685 tidak ditemukan. Kontrol servo akan dinonaktifkan.")
    PCA9685_AVAILABLE = False

# ==============================================================================
# KELAS KONTROLER
# ==============================================================================

class PIDController:
    """Mengelola logika PID untuk satu sumbu (Pan atau Tilt)."""
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0
        self.prev_error = 0

    def calculate(self, error):
        """Menghitung nilai penyesuaian PID berdasarkan error."""
        self.integral += error
        derivative = error - self.prev_error
        adjustment = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error
        return adjustment

    def reset(self):
        """Mereset state integral dan error sebelumnya."""
        self.integral = 0
        self.prev_error = 0

class ServoController:
    """Mengelola inisialisasi dan pergerakan servo."""
    def __init__(self, config):
        if not PCA9685_AVAILABLE:
            self.pwm = None
            return
            
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
        """Mengatur servo ke posisi tengah saat program dimulai."""
        if self.pwm:
            self.pwm.setServoPulse(self.config["pan_channel"], self.config["pan_center_pulse"])
            self.pwm.setServoPulse(self.config["tilt_channel"], self.config["tilt_center_pulse"])

    def update(self, pan_pulse, tilt_pulse):
        """Memperbarui posisi servo dengan pembatasan nilai min/max."""
        if not self.pwm:
            return
            
        clamped_pan = max(self.config["pan_min_pulse"], min(self.config["pan_max_pulse"], pan_pulse))
        clamped_tilt = max(self.config["tilt_min_pulse"], min(self.config["tilt_max_pulse"], tilt_pulse))
        
        try:
            self.pwm.setServoPulse(self.config["pan_channel"], int(clamped_pan))
            self.pwm.setServoPulse(self.config["tilt_channel"], int(clamped_tilt))
        except Exception as e:
            print(f"ERROR: Gagal menggerakkan servo: {e}")

# ==============================================================================
# KONFIGURASI UTAMA (SESUAIKAN DI SINI)
# ==============================================================================

CONFIG = {
    "camera": {
        "width": 640,
        "height": 480
    },
    "servo": {
        "pan_channel": 0,       # Channel untuk servo Pan (kiri-kanan)
        "tilt_channel": 1,      # Channel untuk servo Tilt (atas-bawah)
        "pan_center_pulse": 1500,
        "tilt_center_pulse": 1500,
        "pan_min_pulse": 750,
        "pan_max_pulse": 2250,
        "tilt_min_pulse": 800,
        "tilt_max_pulse": 2200,
        "pan_inversion": 1,     # Ubah ke -1 jika arah Pan terbalik
        "tilt_inversion": -1    # Ubah ke -1 jika arah Tilt terbalik (umumnya tilt terbalik)
    },
    # Nilai Kp, Ki, Kd ini mungkin perlu di-tuning agar gerakan lebih mulus
    "pid_pan":  {"kp": 0.2, "ki": 0.005, "kd": 0.08},
    "pid_tilt": {"kp": 0.25, "ki": 0.005, "kd": 0.1},
    "tracking_tolerance_ratio": 0.05 # Area "aman" di tengah, agar servo tidak bergetar
}

# ==============================================================================
# LOGIKA UTAMA APLIKASI
# ==============================================================================

# --- Inisialisasi Objek Kontroler ---
servo_controller = ServoController(CONFIG)
pid_pan = PIDController(**CONFIG["pid_pan"])
pid_tilt = PIDController(**CONFIG["pid_tilt"])

# --- State Posisi Servo ---
current_pan = float(CONFIG["servo"]["pan_center_pulse"])
current_tilt = float(CONFIG["servo"]["tilt_center_pulse"])

# --- Inisialisasi Kamera ---
picam2 = Picamera2()
cam_config = picam2.create_preview_configuration(main={"size": (CONFIG["camera"]["width"], CONFIG["camera"]["height"])})
picam2.configure(cam_config)
picam2.start()
time.sleep(1.0) # Beri waktu kamera untuk stabil

# --- Inisialisasi Tracker ---
tracker = None
tracking = False
bbox = None

print("INFO: Tekan 's' untuk memilih objek, lalu tekan 'ENTER' atau 'SPACE' untuk memulai tracking.")
print("INFO: Tekan 'q' untuk keluar.")

# --- Loop Utama ---
while True:
    frame = picam2.capture_array()
    # OpenCV dari picamera2 menangkap dalam format BGR, jadi kita ubah ke RGB untuk display
    frame = cv2.cvtColor(frame, cv.COLOR_BGR2RGB) 
    frame = cv2.flip(frame, 1) # Balik frame secara horizontal agar tidak seperti cermin

    frame_height, frame_width, _ = frame.shape
    frame_center_x = frame_width // 2
    frame_center_y = frame_height // 2

    if tracking and tracker is not None:
        success, bbox = tracker.update(frame)

        if success:
            (x, y, w, h) = [int(v) for v in bbox]
            
            # --- LOGIKA KONTROL SERVO ---
            bbox_center_x = x + w // 2
            bbox_center_y = y + h // 2
            
            # Error positif berarti objek ada di 'kanan' atau 'atas' dari pusat.
            error_x = bbox_center_x - frame_center_x
            error_y = frame_center_y - bbox_center_y # Sumbu Y terbalik di koordinat gambar (0 di atas)
            
            tolerance_x = CONFIG["tracking_tolerance_ratio"] * frame_width
            tolerance_y = CONFIG["tracking_tolerance_ratio"] * frame_height

            # Hanya gerakkan servo jika error melebihi toleransi
            if abs(error_x) > tolerance_x or abs(error_y) > tolerance_y:
                adjustment_x = pid_pan.calculate(error_x)
                adjustment_y = pid_tilt.calculate(error_y)
                
                # Gunakan faktor inversi dari config untuk mempermudah pembalikan arah
                current_pan += CONFIG["servo"]["pan_inversion"] * adjustment_x
                current_tilt += CONFIG["servo"]["tilt_inversion"] * adjustment_y
                
                servo_controller.update(current_pan, current_tilt)
            else:
                # Jika sudah di tengah, reset PID agar tidak ada akumulasi error
                pid_pan.reset()
                pid_tilt.reset()

            # --- Visualisasi ---
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (bbox_center_x, bbox_center_y), 5, (255, 0, 0), -1)
            cv2.line(frame, (frame_center_x, frame_center_y), (bbox_center_x, bbox_center_y), (0, 0, 255), 2)
        else:
            cv2.putText(frame, "Tracking Gagal", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            pid_pan.reset()
            pid_tilt.reset()
            tracking = False
            tracker = None

    # Gambar titik pusat frame
    cv2.circle(frame, (frame_center_x, frame_center_y), 5, (0, 255, 255), -1)
    
    cv2.imshow("Object Tracking - CSRT with Servo Control", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('s') and not tracking:
        # Ambil frame baru khusus untuk pemilihan agar tidak blur
        selection_frame = picam2.capture_array()
        selection_frame = cv2.cvtColor(selection_frame, cv2.COLOR_BGR2RGB)
        selection_frame = cv2.flip(selection_frame, 1)

        bbox = cv2.selectROI("Select Object", selection_frame, fromCenter=False, showCrosshair=True)
        cv2.destroyWindow("Select Object")
        
        if bbox[2] > 0 and bbox[3] > 0:
            tracker = cv2.TrackerCSRT_create()
            tracker.init(selection_frame, bbox)
            tracking = True
            pid_pan.reset()
            pid_tilt.reset()
        else:
            print("PERINGATAN: Seleksi tidak valid, objek tidak dipilih.")

    elif key == ord('q'):
        break

# --- Cleanup ---
print("INFO: Menutup program...")
if servo_controller.pwm:
    servo_controller.set_initial_position()
picam2.stop()
cv2.destroyAllWindows()