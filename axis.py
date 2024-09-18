import smbus
import time
import math

# Adresse I2C du MPU9250
MPU9250_ADDRESS = 0x68

# Registres de configuration
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

def initialize_sensor(bus):
    try:
        bus.write_byte_data(MPU9250_ADDRESS, PWR_MGMT_1, 0x00)
        time.sleep(0.1)  # Pause pour laisser le temps au capteur de se réveiller
    except OSError as e:
        print(f"Failed to wake up the MPU9250: {e}")
        raise

def read_sensor_data(bus, register):
    try:
        high = bus.read_byte_data(MPU9250_ADDRESS, register)
        low = bus.read_byte_data(MPU9250_ADDRESS, register + 1)
        value = (high << 8) | low
        if value > 32768:
            value -= 65536
        return value
    except OSError as e:
        print(f"Failed to read from register {register}: {e}")
        raise

def calculate_yaw_pitch_roll(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, dt, yaw, pitch, roll):
    # Convertir les valeurs de gyroscope de degrés par seconde à radians par seconde
    gyro_x_rad = math.radians(gyro_x / 131.0)
    gyro_y_rad = math.radians(gyro_y / 131.0)
    gyro_z_rad = math.radians(gyro_z / 131.0)

    # Calculer les angles d'inclinaison à partir de l'accéléromètre
    accel_pitch = math.atan2(accel_y, accel_z)
    accel_roll = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))

    # Intégrer les vitesses angulaires pour obtenir les angles
    pitch += gyro_x_rad * dt
    roll += gyro_y_rad * dt
    yaw += gyro_z_rad * dt

    # Fusionner les angles obtenus par l'accéléromètre et le gyroscope pour réduire la dérive
    pitch = pitch * 0.98 + accel_pitch * 0.02
    roll = roll * 0.98 + accel_roll * 0.02

    return yaw, pitch, roll

def main():
    bus = None
    try:
        bus = smbus.SMBus(3)  # Change to the correct I2C bus number if needed

        # Vérifier que le capteur est accessible
        try:
            bus.read_byte(MPU9250_ADDRESS)
        except OSError as e:
            print(f"MPU9250 not found at address {MPU9250_ADDRESS}: {e}")
            return

        initialize_sensor(bus)

        yaw, pitch, roll = 0.0, 0.0, 0.0
        prev_time = time.time()

        while True:
            try:
                current_time = time.time()
                dt = current_time - prev_time
                prev_time = current_time

                accel_x = read_sensor_data(bus, ACCEL_XOUT_H)
                accel_y = read_sensor_data(bus, ACCEL_XOUT_H + 2)
                accel_z = read_sensor_data(bus, ACCEL_XOUT_H + 4)

                gyro_x = read_sensor_data(bus, GYRO_XOUT_H)
                gyro_y = read_sensor_data(bus, GYRO_XOUT_H + 2)
                gyro_z = read_sensor_data(bus, GYRO_XOUT_H + 4)

                yaw, pitch, roll = calculate_yaw_pitch_roll(
                    accel_x, accel_y, accel_z,
                    gyro_x, gyro_y, gyro_z,
                    dt, yaw, pitch, roll
                )

                print(f"Yaw: {math.degrees(yaw):.2f}, Pitch: {math.degrees(pitch):.2f}, Roll: {math.degrees(roll):.2f}")

                time.sleep(0.1)
            except Exception as e:
                print(f"An error occurred during sensor reading: {e}")
                break

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        if bus:
            bus.close()

if __name__ == "__main__":
    main()