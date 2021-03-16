#!/usr/bin/env python3

import rospy
import time
import serial
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu

if __name__ == '__main__':
    rospy.init_node('imu_node',anonymous=True)
    rospy.loginfo('Start imu')
    pub = rospy.Publisher('imu', Imu, queue_size=10)
    imu = Imu()
    r = rospy.Rate(20) #20Hz

    s = serial.Serial('/dev/ttyUSB-Arduino', 115200, timeout= 0.1)

    imu.header.seq = 0
    count = 0
    imu.orientation.w = 0.0
    imu.orientation.x = 0.0
    imu.orientation.y = 0.0
    imu.orientation.z = 0.0
    while not rospy.is_shutdown():

        v=[]
        ss = s.readline().decode('utf-8').split(',')
        for i,x in enumerate(ss):
             v.append(float(x))
        #print(v)
        imu.header.seq +=1
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = "refro_imu"
        if len(v) == 4: 
            imu.orientation.w = v[0]
            imu.orientation.x = v[1]
            imu.orientation.y = v[2]
            imu.orientation.z = v[3]
        pub.publish(imu)

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
