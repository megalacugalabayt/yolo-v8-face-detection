import os
from time import sleep
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

def set_mode(master, mode):
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"Mode set to {mode}.")
    sleep(2)  # Give some time for mode to be set

def wait_for_altitude(master, target_altitude):
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg.relative_alt / 1000.0 >= target_altitude:
            print(f"Reached target altitude: {target_altitude} meters.")
            break
        sleep(1)

def maintain_thrust_until_altitude(master, target_altitude):
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg.relative_alt / 1000.0 >= target_altitude:
            print(f"Reached target altitude: {target_altitude} meters.")
            break
        master.mav.manual_control_send(
            master.target_system,
            0,  # Roll
            0,  # Pitch
            1000,  # Throttle (1000 is full throttle in manual control)
            0,  # Yaw
            0   # Buttons
        )
        sleep(0.1)  # Check altitude frequently

def land(master):
    set_mode(master, 'LAND')
    print("Landing mode activated.")
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg.relative_alt / 1000.0 <= 0.1:  # Close to ground
            print("Landed.")
            break
        sleep(1)

def print_motor_outputs(master):
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER, 1, 1)
    message = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=1)
    if message:
        print(f"Motor outputs: {message.servo1_raw}, {message.servo2_raw}, {message.servo3_raw}, {message.servo4_raw}")

def main():
    connection_string = 'COM7'
    master = connect_to_pixhawk(connection_string)

    if master is None:
        print("Failed to connect to Pixhawk.")
        return

    arm_vehicle(master)
    set_mode(master, 'THROW')  # Set to THROW mode
    print("Waiting for the drone to take off in THROW mode.")

    maintain_thrust_until_altitude(master, 1)  # Maintain thrust until the drone reaches 1 meter altitude

    print("Hovering for 5 seconds.")
    for _ in range(5):  # Hover for 5 seconds
        print_motor_outputs(master)
        sleep(1)

    land(master)  # Set to LAND mode and wait for the drone to land

    master.close()
    print("Disconnected from Pixhawk.")

if __name__ == "__main__":
    main()
