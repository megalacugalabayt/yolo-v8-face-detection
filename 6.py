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

# Dronu arm etmek için fonksiyon
def arm_vehicle(master):
    print("Dron arm ediliyor...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Dron arm edildi.")

# MAVLink Motor Kontrol Komutları Gönderme ve Terminale Yazdırma
def send_motor_command(master, motor1, motor2, motor3, motor4):
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        motor1,  # Motor 1 (Roll pozitif)
        motor2,  # Motor 2 (Pitch pozitif)
        motor3,  # Motor 3 (Roll negatif)
        motor4,  # Motor 4 (Pitch negatif)
        0, 0, 0, 0)  # Diğer kanallar (kullanılmıyor)
    # Gönderilen komutları terminale yazdır
    print(f"Motor komutu gönderildi -> Motor1: {motor1}, Motor2: {motor2}, Motor3: {motor3}, Motor4: {motor4}")

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
        motor1 = 1500  # Sabit throttle değeri
        motor2 = 1500
        motor3 = 1500
        motor4 = 1500

        if face_center_x < frame_center_x - 50:
            motor1 += 50  # Motor 1 hızını artır (Roll pozitif)
            motor3 -= 50  # Motor 3 hızını azalt (Roll negatif)
        elif face_center_x > frame_center_x + 50:
            motor1 -= 50  # Motor 1 hızını azalt (Roll pozitif)
            motor3 += 50  # Motor 3 hızını artır (Roll negatif)
        
        if face_center_y < frame_center_y - 50:
            motor2 += 50  # Motor 2 hızını artır (Pitch pozitif)
            motor4 -= 50  # Motor 4 hızını azalt (Pitch negatif)
        elif face_center_y > frame_center_y + 50:
            motor2 -= 50  # Motor 2 hızını azalt (Pitch pozitif)
            motor4 += 50  # Motor 4 hızını artır (Pitch negatif)

        send_motor_command(master, motor1, motor2, motor3, motor4)

# Ana Fonksiyon
def main():
    # USB bağlantısı için connection_string belirle
    connection_string = 'COM7'  # Pixhawk USB bağlantı noktası (Windows)
    master = connect_to_pixhawk(connection_string)

    # Dronu arm et
    arm_vehicle(master)

    # Video Akışı Başlat
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()  # Kamera görüntüsünü al
        if not ret:
            print("Kamera akışında hata oluştu.")
            break
        
        process_frame(master, frame)

        cv2.imshow('frame', frame)  # Görüntüyü göster
        
        if cv2.waitKey(1) & 0xFF == ord('e'):
            print("'e' tuşuna basıldı. Dron disarm ediliyor ve bağlantı kapatılıyor...")
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
