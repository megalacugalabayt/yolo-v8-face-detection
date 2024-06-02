import os
import cv2
from pymavlink import mavutil
from ultralytics import YOLO

# Set the environment variable for the Qt platform plugin
os.environ['QT_QPA_PLATFORM'] = 'xcb'

# Load YOLO model trained for face detection
model = YOLO("best.pt")

def connect_to_pixhawk(connection_string, baudrate=115200):
    master = mavutil.mavlink_connection(connection_string, baud=baudrate)
    master.wait_heartbeat()
    print("Connected to Pixhawk.")
    return master

def arm_vehicle(master):
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Drone is armed.")

def send_manual_control(master, x, y, z, r):
    master.mav.manual_control_send(
        master.target_system,
        x,  # Roll
        y,  # Pitch
        z,  # Throttle
        r,  # Yaw
        0   # Buttons
    )

def process_frame(master, frame):
    # Perform inference with YOLO model
    results = model(frame)
    frame_center_x = frame.shape[1] // 2
    detections = results[0].boxes.xyxy.cpu().numpy()  # Get bounding boxes in xyxy format

    if len(detections) > 0:
        # Assuming the first detected face is the one to track
        x1, y1, x2, y2 = detections[0][:4]
        face_center_x = int((x1 + x2) / 2)

        yaw = 0
        if face_center_x < frame_center_x - 50:
            yaw = -100  # Turn left
        elif face_center_x > frame_center_x + 50:
            yaw = 100   # Turn right

        print(f"Face center: {face_center_x}, Frame center: {frame_center_x}, Yaw command: {yaw}")

        # Maintain constant throttle and no roll or pitch
        send_manual_control(master, 0, 0, 100, yaw)
    else:
        print("No faces detected.")

def main():
    connection_string = 'COM7'
    master = connect_to_pixhawk(connection_string)

    if master is None:
        print("Failed to connect to Pixhawk.")
        return

    arm_vehicle(master)
    
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Failed to open camera.")
        return

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            process_frame(master, frame)
            cv2.imshow('frame', frame)
            
            if cv2.waitKey(1) & 0xFF == ord('e'):
                break

    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
