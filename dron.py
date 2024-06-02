import cv2
import time
from pymavlink import mavutil

# OpenCV Yüz Algılama İçin Haar Cascades Yükle
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# MAVLink Bağlantısı Kur
def connect_to_pixhawk(connection_string):
    master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    print("Pixhawk ile bağlantı kuruldu.")
    return master

# MAVLink Komutları Gönderme ve Terminale Yazdırma
def send_mavlink_command(master, roll, pitch, yaw, throttle):
    master.mav.manual_control_send(
        master.target_system,
        roll,   # Roll komutu
        pitch,  # Pitch komutu
        yaw,    # Yaw komutu
        throttle,  # Throttle komutu
        0)  # Butonlar (kullanılmıyor)
    # Gönderilen komutları terminale yazdır
    print(f"Komut gönderildi -> Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}, Throttle: {throttle}")

# Yüz Algılama ve Kontrol Fonksiyonu
def process_frame(master, frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Görüntüyü gri tonlamaya çevir
    faces = face_cascade.detectMultiScale(gray, 1.1, 4)  # Yüzleri algıla

    if len(faces) > 0:
        # İlk Yüzü Al
        (x, y, w, h) = faces[0]
        face_center_x = x + w // 2
        face_center_y = y + h // 2

        # Görüntünün Merkezini Hesapla
        frame_center_x = frame.shape[1] // 2
        frame_center_y = frame.shape[0] // 2

        # Yüzü Ortalamak İçin Hareket Komutlarını Hesapla
        roll = 0
        pitch = 0
        yaw = 0
        throttle = 1500  # Sabit throttle değeri

        if face_center_x < frame_center_x - 50:
            yaw = -50  # Sola dön
        elif face_center_x > frame_center_x + 50:
            yaw = 50  # Sağa dön
        
        if face_center_y < frame_center_y - 50:
            pitch = 50  # Yukarı çık
        elif face_center_y > frame_center_y + 50:
            pitch = -50  # Aşağı in

        send_mavlink_command(master, roll, pitch, yaw, throttle)

# Ana Fonksiyon
def main():
    # USB bağlantısı için connection_string belirle
    connection_string = 'COM7'  # Pixhawk USB bağlantı noktası (Windows)
    master = connect_to_pixhawk(connection_string)

    # Video Akışı Başlat
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()  # Kamera görüntüsünü al
        if not ret:
            print("Kamera akışında hata oluştu.")
            break
        
        process_frame(master, frame)

        cv2.imshow('frame', frame)  # Görüntüyü göster
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()