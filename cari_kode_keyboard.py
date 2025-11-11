import cv2
import numpy as np

# Buat gambar hitam kosong sebagai jendela
img = np.zeros((200, 400), np.uint8)
window_name = "Pencari Kode Tombol (Tekan 'q' untuk keluar)"
cv2.imshow(window_name, img)

print("Jendela 'Pencari Kode Tombol' aktif.")
print("Tekan tombol apa saja pada keyboard Anda untuk melihat kodenya.")
print("---------------------------------------------------------")

while True:
    # Tunggu penekanan tombol tanpa batas waktu
    key = cv2.waitKey(0)

    if key == -1:
        # Lanjutkan jika tidak ada tombol yang ditekan (seharusnya tidak terjadi dengan waitKey(0))
        continue
    
    # Cetak kode tombol yang terdeteksi
    print(f"Tombol Ditekan! Kodenya adalah: {key}")

    # Keluar dari loop jika tombol 'q' ditekan
    if key == ord('q'):
        print("Keluar dari program.")
        break

cv2.destroyAllWindows()