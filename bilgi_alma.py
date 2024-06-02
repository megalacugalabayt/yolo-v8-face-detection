from dronekit import connect, VehicleMode
import time

# Bağlantı stringini COM portuna ayarla
connection_string = 'COM7'
baud_rate = 57600

print(">>>> Connecting with the UAV <<<")
vehicle = connect(connection_string, wait_ready=True, baud=baud_rate)

try:
    while True:
        # Bilgileri oku ve yazdır
        print('Autopilot version: %s' % vehicle.version)
        print('Supports set attitude from companion: %s' % vehicle.capabilities.set_attitude_target_local_ned)
        print('Position: %s' % vehicle.location.global_relative_frame)
        print('Attitude: %s' % vehicle.attitude)
        print('Velocity: %s' % vehicle.velocity)
        print('Last Heartbeat: %s' % vehicle.last_heartbeat)
        print('Is the vehicle armable: %s' % vehicle.is_armable)
        print('Groundspeed: %s' % vehicle.groundspeed)
        print('Mode: %s' % vehicle.mode.name)
        print('Armed: %s' % vehicle.armed)
        print('EKF Ok: %s' % vehicle.ekf_ok)

        time.sleep(1)  # 1 saniye bekle

except KeyboardInterrupt:
    print("Program sonlandırıldı. Bağlantı kapatılıyor.")

finally:
    # Bağlantıyı kapat
    vehicle.close()
    print("Connection closed.")

