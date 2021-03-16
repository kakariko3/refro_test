#!/usr/bin/env python
import rospy
#import serial
#import struct
import time
from threading import Thread
import threading
from math import sin,cos,pi

from tf import transformations, TransformListener
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from geometry_msgs.msg import Twist

WHEEL_D = 0.15
WIDTH = 0.408
GEAR_RATIO = 15
PULSE = 2048

INC_ST_ACC = 0.006
INC_RT_ACC = 0.006

TTYACM_L = '/dev/ttyACM_MOTOR_L'
TTYACM_R = '/dev/ttyACM_MOTOR_R'

SERIAL_RATE = 115200
SEND_HZ = 50
SEND_PERIOD = 0.004

BASE_FRAME_ID = '/base_link'
ODOM_FRAME_ID = 'odom'

STX = 0x02
ETX = 0x03

GET = 0BFA001
SET = 0BFA002

GET_SV_STATUS   = chr(STX) + GET + '14' + '01EF'     + chr(ETX)
GET_VEL_FDBACK  = chr(STX) + GET + '29' + '01F5'     + chr(ETX)
SET_VEL_CTL     = chr(STX) + SET + '4B' + '000261'   + chr(ETX)
SET_VEL_MODE    = chr(STX) + SET + '1F' + '020264'   + chr(ETX)
SET_SV_ON       = chr(STX) + SET + '1E' + '010262'   + chr(ETX)
SET_SV_OFF      = chr(STX) + SET + '1E' + '000261'   + chr(ETX)
SET_VEL_TARGET  = chr(STX) + SET + '25'
SET_VEL_ZERO    = chr(STX) + SET + '25' + '000002B2' + chr(ETX)

LEFT   = 0
RIGHT  = 1
ALL    = 2
ERROR  = 1
SV_OFF = 0
SV_ON  = 1

SEND_HZ = 20 

class main():
    def __init__(self):
        rospy.Subscriber('/normal_vel', Twist, self.vel_handler, queue_size=10)

        self.pub_odom = rospy.Publisher("/odom", Odometry, queue_size=10)

        ## cmd_vel info
        self.vel_l = 0.0 #m/s
        self.vel_r = 0.0 #m/s
        self.last_vel_l = 0.0
        self.last_vel_r = 0.0
        self.vel_flag = True
        self.latest_vel_time = rospy.Time.now() # 最後にnon-zeroのcmd_velをsubscribeした時間

        ## servo info
        self.sv_on_l = False
        self.sv_on_r = False
        self.sv_on = False
        self.force_end = False
        self.force_sv_off = False
        self.actual_sv_status = False
        
        ## Serial
        self.ser_left = None #serial.Serial(self.serial_name_left, self.serial_baud_rate, timeout=0.1)
        self.ser_right = None #serial.Serial(self.serial_name_right, self.serial_baud_rate, timeout=0.1)

        ## Tread
        self.lock = threading.Lock()
        self.thread_flag = True
        #self.send_thread = Thread(target=self.send_thread_fn)
        self.send_thread.setDaemon(True)
        self.send_thread.start()
        #self.receive_thread = Thread(target=self.receive_thread_fn)
        self.receive_thread.setDaemon(True)
        self.receive_thread.start()

    def receive_thread_fn(self): # ServoDriver to PC
        rate = rospy.Rate(SEND_HZ)
        while not rospy.is_shutdown():

            if self.thread_flag == False:
                break

    def send_thread_fn(self): # PC to ServoDriver
        
        rate = rospy.Rate(SEND_HZ)
        while not rospy.is_shutdown():
            rate.sleep()

            if self.sv_on == False and self.force_sv_off == False:

                if self.ser_left == None or self.ser_right == None or self.ser_left.is_open == False or self.ser_right.is_open == False:
                    #self.ser_left = serial.Serial(self.serial_name_left, self.serial_baud_rate, timeout=0.1)
                    #self.ser_right = serial.Serial(self.serial_name_right, self.serial_baud_rate, timeout=0.1)

                if self.do_servo_on() == False:
                    loginfo('servo on command was failed')
                    continue
                loginfo('servo on!!')
                self.servo_on = True

            if self.servo_on == False:
                continue

            if self.actual_servo_status == False:
                self.do_servo_on()
                continue

            if self.force_end == True:
                break

    def vel_handler(self, msg):
        x = msg.linear.x
        z = msg.angular.z
        if x != 0.0 or z != 0.0:
            self.latest_vel_time = rospy.Time.now()
            if self.force_sv_off == True:
                self.force_sv_off = False
        else:
            duration = rospy.Time.now() - self.latest_vel_time 
            if duration >= rospy.Duration(self.no_cmd_vel_time_thresh):
                if self.force_sv_off == False:
                    self.force_sv_off = True

        self.vel_l = x - z * self.width / 2
        self.vel_r = x + z * self.width / 2

if __name__ == '__main__':
    rospy.init_node('move_node',anonymous=True)
    rospy.loginfo('Start move_node')
    r = rospy.Rate(20) #20Hz
    try:
        m = main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
