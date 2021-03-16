#!/usr/bin/env python3

# https://github.com/adafruit/Adafruit_Python_BNO055
# sudo pip3 install adafruit-circuitpython-bno055

import rospy
import time
import serial
import adafruit_bno055
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu

if __name__ == '__main__':
    rospy.init_node('imu_node',anonymous=True)
    rospy.loginfo('Start imu')
    pub = rospy.Publisher('imu', Imu, queue_size=10)
    imu = Imu()
    r = rospy.Rate(20) #20Hz

    # User these lines for UART
    uart = serial.Serial('/dev/ttyUSB-FT232R', 192000, timeout=1.0)
    sensor = adafruit_bno055.BNO055_UART(uart)

    imu.header.seq = 0
    count = 0
    imu.orientation.w = 0.0
    imu.orientation.x = 0.0
    imu.orientation.y = 0.0
    imu.orientation.z = 0.0
    while not rospy.is_shutdown():
        imu.header.seq +=1
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = "refro_imu"
        try:
            val = sensor.quaternion
            imu.orientation.w = val[0] #sensor.quaternion[0]
            imu.orientation.x = val[1] #sensor.quaternion[1]
            imu.orientation.y = val[2] #sensor.quaternion[2]
            imu.orientation.z = val[3] #sensor.quaternion[3]
            #print("eular: {}",format(sensor.eular))
            val2 = sensor.euler
        except:
            continue
        print(val2)
        pub.publish(imu)
        """
        print("Temperature: {} degrees C".format(sensor.temperature))
        print("Accelerometer (m/s^2): {}".format(sensor.acceleration))
        print("Magnetometer (microteslas): {}".format(sensor.magnetic))
        print("Gyroscope (rad/sec): {}".format(sensor.gyro))
        print("Euler angle: {}".format(sensor.euler))
        print("Quaternion: {}".format(sensor.quaternion))
        print("Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration))
        print("Gravity (m/s^2): {}".format(sensor.gravity))
        print()
        """ 
        """       
        imu.header.seq +=1
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = "refro_imu"
        imu.orientation.w = sensor.quaternion[0]
        imu.orientation.x = sensor.quaternion[1]
        imu.orientation.y = sensor.quaternion[2]
        imu.orientation.z = sensor.quaternion[3]
        imu.angular_velocity.x = sensor.gyro[0]
        imu.angular_velocity.y = sensor.gyro[1]
        imu.angular_velocity.z = sensor.gyro[2]
        imu.linear_acceleration.x = sensor.acceleration[0]
        imu.linear_acceleration.y = sensor.acceleration[1]
        imu.linear_acceleration.z = sensor.acceleration[2]
        """
        r.sleep()
