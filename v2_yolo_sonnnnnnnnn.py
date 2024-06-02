import cv2
import csv
import time
from ultralytics import YOLO

def determine_object_position(x1, y1, x2, y2, frame_width, frame_height):
    # Nesnenin merkezini bul
    center_x = (x1 + x2) // 2
    center_y = (y1 + y2) // 2
    
    # Orijine olan uzaklıkları x ve y ekseninde ayrı ayrı hesapla
    distance_x = abs(center_x - frame_width // 2)
    distance_y = abs(center_y - frame_height // 2)
    
    # Dikey konumu belirle (yukarıda mı aşağıda mı)
    vertical_position = "Aşağıda" if center_y > frame_height // 2 else "Yukarıda"
    
    # Yatay konumu belirle (sağda mı solda mı)
    horizontal_position = "Sağda" if center_x > frame_width // 2 else "Solda"
    
    return distance_x, distance_y, vertical_position, horizontal_position

# YOLO modelini yükle
model = YOLO("best.pt")

# Kamera yakalama başlat
cap = cv2.VideoCapture(0)

# Kamera çözünürlüğünü al
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# CSV dosyasını yazma modunda aç
output_filename = "object_distances.csv"
with open(output_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Frame", "Distance_X", "Horizontal_Position", "Distance_Y", "Vertical_Position"])  # Başlık satırını düzelt

frame_count = 0
last_write_time = time.time()

while cap.isOpened():
    ret, img = cap.read()
    if not ret:
        break

    # Görüntüyü x ekseni etrafında çevir
    img = cv2.flip(img, 1)  # 1, yatayda ters çevirir
    
    # YOLO modeli için görüntüyü yeniden boyutlandır
    img_resized = cv2.resize(img, (800, 600))
    frame_count += 1
    
    results = model(img_resized, stream=True)

    try:
        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Bounding Box
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                
                # Nesne pozisyonunu belirle
                distance_x, distance_y, vertical_position, horizontal_position = determine_object_position(x1, y1, x2, y2, img_resized.shape[1], img_resized.shape[0])

                # Belirli aralıklarla CSV dosyasına yaz
                current_time = time.time()
                if current_time - last_write_time >= 0.5:  # Her 1 saniyede bir yaz
                    with open(output_filename, mode='a', newline='') as file:
                        writer = csv.writer(file)
                        writer.writerow([frame_count, distance_x, horizontal_position, distance_y, vertical_position])
                    last_write_time = current_time

                # Çerçeveye nesne sınırlayıcı kutusunu (bounding box) ve pozisyonu çiz
                cv2.rectangle(img_resized, (x1, y1), (x2, y2), (255, 0, 255), 3)  # Mor renk
                
                # Nesne merkezine kırmızı bir nokta çiz
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                cv2.circle(img_resized, (center_x, center_y), 5, (0, 0, 255), -1)  # Kırmızı renk

        # Ekranı dört bölgeye bölen çizgileri çiz
        cv2.line(img_resized, (img_resized.shape[1]//2, 0), (img_resized.shape[1]//2, img_resized.shape[0]), (0, 255, 0), 2)  # Dikey çizgi
        cv2.line(img_resized, (0, img_resized.shape[0]//2), (img_resized.shape[1], img_resized.shape[0]//2), (0, 255, 0), 2)  # Yatay çizgi

        cv2.imshow('img', img_resized)
        a = cv2.waitKey(1)
        if a == ord('q'):
            break
    except cv2.error:
        print("Stream bitti")
        break

cap.release()
cv2.destroyAllWindows()

print(f"Nesne mesafeleri ve pozisyonları, {output_filename} dosyasına yazıldı.")
