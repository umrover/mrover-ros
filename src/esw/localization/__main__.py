import rospy
from sensor_msgs.msg import Imu
import serial
from serial import SerialException


def main():
    rospy.init_node("imu_driver")
    pub = rospy.Publisher("imu", Imu)

    ser = serial.Serial('/dev/imu', 115200)
    attempts = 0

    while True:
        imu_msg = Imu()
        try:
            line = ser.readline()
            attempts = 0
        except SerialException:
            attempts += 1
            if attempts > 100:
                print("unable to read from serial port...ending program")
                break

        try:
            vals = [float(val.strip()) for val in line.split()]
        except ValueError:
            vals = []
            print("invalid msg format, make sure DEBUG mode isn't enabled in the arduino sketch")

        print("vals: ", vals)
        try:
            # imu_data.accel_x_g = vals[0]
            # imu_data.accel_y_g = vals[1]
            # imu_data.accel_z_g = vals[2]

            # imu_data.gyro_x_dps = vals[3]
            # imu_data.gyro_y_dps = vals[4]
            # imu_data.gyro_z_dps = vals[5]

            # imu_data.mag_x_uT = vals[6]
            # imu_data.mag_y_uT = vals[7]
            # imu_data.mag_z_uT = vals[8]

            # imu_data.calibration_sys = vals[9]
            # imu_data.calibration_gyro = vals[10]
            # imu_data.calibration_accel = vals[11]
            # imu_data.calibration_mag = vals[12]

            # imu_data.bearing_deg = vals[13]
            imu_msg.orientation.x

        except IndexError:
            print("incomplete msg")


if __name__ == "__main__":
    main()
