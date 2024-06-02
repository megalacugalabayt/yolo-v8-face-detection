import cv2
from pymavlink import mavutil

# Load Haar Cascades for face detection
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def connect_to_pixhawk(connection_string, baudrate=115200):
    master = mavutil.mavlink_connection(connection_string, baudrate)
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
    print(f"Sent yaw control: {r}")

def process_frame(master, frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.1, 4)

    frame_width = frame.shape[1]
    frame_height = frame.shape[0]
    frame_center_x = frame_width // 2
    frame_center_y = frame_height // 2

    # Divide the screen into a grid
    cv2.line(frame, (frame_center_x, 0), (frame_center_x, frame_height), (0, 255, 0), 2)
    cv2.line(frame, (0, frame_center_y), (frame_width, frame_center_y), (0, 255, 0), 2)

    if len(faces) > 0:
        x, y, w, h = faces[0]
        face_center_x = x + w // 2
        face_center_y = y + h // 2

        yaw = 0
        # Horizontal control
        if face_center_x < frame_center_x - 50:
            yaw = -50  # Softer turn left
        elif face_center_x > frame_center_x + 50:
            yaw = 50   # Softer turn right

        # Vertical control (for future use)
        pitch = 0
        if face_center_y < frame_center_y - 50:
            pitch = -50  # Move up
        elif face_center_y > frame_center_y + 50:
            pitch = 50   # Move down

        send_manual_control(master, 0, pitch, 100, yaw)

        # Draw a rectangle and line for visualization
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.line(frame, (face_center_x, face_center_y), (frame_center_x, frame_center_y), (255, 0, 0), 2)

    else:
        print("No face detected.")

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

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame.")
            break

        process_frame(master, frame)
        cv2.imshow('frame', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('e'):
            print("Exiting...")
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
