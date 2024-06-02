import time
from pymavlink import mavutil

# MAVLink Bağlantısı Kur
def connect_to_pixhawk(connection_string, baudrate):
    master = mavutil.mavlink_connection(connection_string, baud=baudrate)
    master.wait_heartbeat()
    print("Pixhawk ile bağlantı kuruldu.")
    return master

# Dronu arm etmek ve disarm etmek için fonksiyonlar
def arm_vehicle(master):
    print("Dron arm ediliyor...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Dron arm edildi.")

def disarm_vehicle(master):
    print("Dron disarm ediliyor...")
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print("Dron disarm edildi.")

# Takeoff işlemi için fonksiyon
def takeoff(master, altitude):
    print(f"{altitude} metre yüksekliğe kalkış yapılıyor...")
    master.mav.command_long_send(
        master.target_system, 
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )
    time.sleep(10)  # Kalkış işlemi için bekle

# Mod değiştirme fonksiyonu
def set_mode(master, mode):
    # Uçuş modunu belirlemek için mod numarasını kullanın
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"Uçuş modu {mode} olarak ayarlandı.")
    time.sleep(1)

# Ana fonksiyon
def main():
    connection_string = 'COM7'  # Pixhawk USB bağlantı noktası
    baudrate = 921600  # Pixhawk baudrate
    target_altitude = 10  # Hedef irtifa (metre)

    master = connect_to_pixhawk(connection_string, baudrate)
    try:
        arm_vehicle(master)
        
        # Stabilize modunda kalkış
        set_mode(master, 'STABILIZE')
        takeoff(master, target_altitude)

        # Loiter moduna geçiş ve sabit kalma
        set_mode(master, 'LOITER')
        print(f"Loiter modunda {target_altitude} metre yüksekliğinde sabitleniyor.")
        
        # 'q' tuşuna basılmasını bekle
        print("Dron disarm edilmesi için 'q' tuşuna basın.")
        while True:
            if input() == 'q':
                print("'q' tuşuna basıldı. Dron disarm ediliyor...")
                break
            time.sleep(1)

    finally:
        disarm_vehicle(master)
        master.close()  # Bağlantıyı kapat
        print("Bağlantı kapatıldı.")

if __name__ == "__main__":
    main()
