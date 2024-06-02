from pymavlink import mavutil
import cv2
import time
import threading

# Initialize the last_command_time at the start of the script
last_command_time = time.time()  # Global variable for tracking command timing

def connect_to_pixhawk(connection_string, baudrate=115200):
    try:
        master = mavutil.mavlink_connection(connection_string, baudrate)
        master.wait_heartbeat()
        print("Connection to Pixhawk established.")
        return master
    except Exception as e:
        print(f"Connection error: {e}")
        return None

def arm_vehicle(master):
    try:
        print("Arming the drone...")
        master.arducopter_arm()
        master.motors_armed_wait()
        print("Drone armed.")
    except Exception as e:
        print(f"Arming error: {e}")

def send_motor_command(master, throttle, yaw):
    try:
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            1500,  # Aileron (roll)
            1500,  # Elevator (pitch)
            throttle,  # Throttle
            yaw,  # Rudder (yaw)
            0, 0, 0, 0)  # Remaining channels
        print(f"Motor command sent -> Throttle: {throttle}, Yaw: {yaw}")
    except Exception as e:
        print(f"Error sending motor command: {e}")

def process_frame(frame, master, frame_center_x, frame_center_y, face_cascade):
    global last_command_time
    current_time = time.time()
    if current_time - last_command_time < 5:
        return  # Skip processing if within 5 seconds of the last command

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    if len(faces) > 0:
        x, y, w, h = faces[0]
        face_center_x, face_center_y = x + w // 2, y + h // 2
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.circle(frame, (face_center_x, face_center_y), 5, (255, 0, 0), -1)

        throttle = 1200  # Neutral throttle
        yaw = 1500  # Neutral yaw
        if face_center_x > frame_center_x + 50:  # Adjust yaw based on the face position
            yaw = 1600
        elif face_center_x < frame_center_x - 50:
            yaw = 1400

        send_motor_command(master, throttle, yaw)
        last_command_time = current_time

def camera_thread(master):
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)  # Set video width
    cap.set(4, 480)  # Set video height
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    while True:
        ret, frame = cap.read()
        if ret:
            frame_height, frame_width = frame.shape[:2]
            frame_center_x, frame_center_y = frame_width // 2, frame_height // 2
            process_frame(frame, master, frame_center_x, frame_center_y, face_cascade)
            cv2.imshow('Frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    cap.release()
    cv2.destroyAllWindows()

def main():
    master = connect_to_pixhawk('com7', 57600)
    if not master:
        return
    arm_vehicle(master)
    cam_thread = threading.Thread(target=camera_thread, args=(master,))
    cam_thread.start()
    cam_thread.join()
    master.close()
    print("Connection closed")

if __name__ == "__main__":
    main()
