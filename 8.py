import cv2
import time
from pymavlink import mavutil

# OpenCV Yüz Algılama İçin Haar Cascades Yükle
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# MAVLink Bağlantısı Kur
def connect_to_pixhawk(connection_string, baudrate=115200):
    try:
        master = mavutil.mavlink_connection(connection_string, baud=baudrate)
        master.wait_heartbeat()
        print("Pixhawk ile bağlantı kuruldu.")
        return master
    except Exception as e:
        print(f"Bağlantı hatası: {e}")
        return None

# Dronu arm etmek için fonksiyon
def arm_vehicle(master):
    try:
        print("Dron arm ediliyor...")
        master.arducopter_arm()
        master.motors_armed_wait()
        print("Dron arm edildi.")
    except Exception as e:
        print(f"Arm etme hatası: {e}")

# MAVLink Motor Kontrol Komutları Gönderme ve Terminale Yazdırma
def send_motor_command(master, motor1, motor2, motor3, motor4):
    try:
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
    except Exception as e:
        print(f"Motor komutu gönderme hatası: {e}")
        # Hata durumunda motorları sabit değerde çalıştırmaya devam et
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            1100,  # Motor 1
            1100,  # Motor 2
            1100,  # Motor 3
            1100,  # Motor 4
            0, 0, 0, 0)
        print("Varsayılan motor komutları gönderildi.")

# Yüz Algılama ve Kontrol Fonksiyonu
def process_frame(master, frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Görüntüyü gri tonlamaya çevir
    faces = face_cascade.detectMultiScale(gray, 1.1, 4)  # Yüzleri algıla

    # Ekranı x ve y eksenlerinde dört eşit parçaya böl
    frame_height, frame_width = frame.shape[:2]
    frame_center_x = frame_width // 2
    frame_center_y = frame_height // 2

    cv2.line(frame, (frame_center_x, 0), (frame_center_x, frame_height), (0, 255, 0), 2)
    cv2.line(frame, (0, frame_center_y), (frame_width, frame_center_y), (0, 255, 0), 2)

    if len(faces) > 0:
        # İlk Yüzü Al
        (x, y, w, h) = faces[0]
        face_center_x = x + w // 2
        face_center_y = y + h // 2

        # Yüzün etrafına kare çiz
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        # Yüzün merkezine mavi nokta ekle
        cv2.circle(frame, (face_center_x, face_center_y), 5, (255, 0, 0), -1)
        # Yüz merkezinden görüntü merkezine çizgi çiz
        cv2.line(frame, (face_center_x, face_center_y), (frame_center_x, frame_center_y), (255, 0, 0), 2)

        # Yüzü Ortalamak İçin Hareket Komutlarını Hesapla
        motor1 = 1200  # Sabit throttle değeri
        motor2 = 1200
        motor3 = 1200
        motor4 = 1200

        if face_center_x < frame_center_x - 50:
            motor1 = 1050  # Motor 1 hızını düşür
        elif face_center_x > frame_center_x + 50:
            motor2 = 1050  # Motor 3 hızını düşür
        
        if face_center_y < frame_center_y - 50:
            motor3 = 1050  # Motor 2 hızını düşür
        elif face_center_y > frame_center_y + 50:
            motor4 = 1050  # Motor 4 hızını düşür

        send_motor_command(master, motor1, motor2, motor3, motor4)

# Ana Fonksiyon
def main():
    # USB bağlantısı için connection_string belirle
    connection_string = 'COM7'  # Pixhawk USB bağlantı noktası (Windows)
    master = connect_to_pixhawk(connection_string)

    if master is None:
        print("Pixhawk'a bağlanılamadı.")
        return

    # Dronu arm et
    arm_vehicle(master)

    # Video Akışı Başlat
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Kamera açılamadı.")
        return

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
