import os
from pymavlink import mavutil
import time

# Set the environment variable for the Qt platform plugin
os.environ['QT_QPA_PLATFORM'] = 'xcb'

def connect_to_pixhawk(connection_string, baudrate=115200):
    master = mavutil.mavlink_connection(connection_string, baud=baudrate)
    master.wait_heartbeat()
    print("Connected to Pixhawk.")
    return master

def arm_vehicle(master):
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Drone is armed.")

def set_throttle(master, throttle):
    master.mav.manual_control_send(
        master.target_system,
        0,  # Roll
        0,  # Pitch
        throttle,  # Throttle
        0,  # Yaw
        0   # Buttons
    )

def main():
    connection_string = 'COM7'
    master = connect_to_pixhawk(connection_string)

    if master is None:
        print("Failed to connect to Pixhawk.")
        return

    arm_vehicle(master)

    try:
        throttle = 1200
        while throttle <= 1400:
            set_throttle(master, throttle)
            print(f"Throttle set to {throttle}")
            throttle += 20
            time.sleep(1)  # 1 saniye bekle ve throttle değerini arttır

        print("Maintaining throttle at 1400 for 5 seconds")
        set_throttle(master, 1400)
        time.sleep(5)

        print("Switching to landing mode")
        master.set_mode("LAND")

        # İniş için bağlantıyı kontrol ederek 5 saniye bekle
        for _ in range(5):
            try:
                master.wait_heartbeat(timeout=1)
                print("Landing...")
            except mavutil.mavtimeout:
                print("Connection lost. Attempting to reconnect...")
                master = connect_to_pixhawk(connection_string)
                if master is None:
                    print("Reconnection failed.")
                    return

        print("Landing complete")

    except KeyboardInterrupt:
        print("Program interrupted by user.")

if __name__ == "__main__":
    main()
