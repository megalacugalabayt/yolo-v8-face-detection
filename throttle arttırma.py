import os
import time
from pymavlink import mavutil

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

def send_manual_control(master, x, y, z, r):
    master.mav.manual_control_send(
        master.target_system,
        x,  # Roll
        y,  # Pitch
        z,  # Throttle
        r,  # Yaw
        0   # Buttons
    )

def get_altitude(master):
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg is not None:
        return msg.relative_alt / 1000.0  # converting millimeters to meters
    return None

def main():
    connection_string = 'COM7'
    master = connect_to_pixhawk(connection_string)

    if master is None:
        print("Failed to connect to Pixhawk.")
        return

    arm_vehicle(master)

    try:
        throttle = 1
        while throttle <= 200:
            # Send throttle value
            send_manual_control(master, 0, 0, throttle, 0)
            altitude = get_altitude(master)
            print(f"Throttle: {throttle}, Altitude: {altitude} meters")

            # Increment throttle by 100 every second
            throttle += 10

            time.sleep(1)
            
    except KeyboardInterrupt:
        print("Program interrupted by user.")

if __name__ == "__main__":
    main()
