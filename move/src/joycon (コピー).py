#!/usr/bin/env python
import rospy
import math
import Jetson.GPIO as GPIO
import smbus
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

GPIO.setmode(GPIO.BOARD)
L_SV = 16
R_SV = 18
GPIO.setup(L_SV,GPIO.OUT)
GPIO.setup(R_SV,GPIO.OUT)
# Initialize Servo Off
GPIO.output(L_SV,False)
GPIO.output(R_SV,False)

ADR_CTL = 0x20
ADR_VEL_L = 0x60
ADR_VEL_R = 0x61

L_CW = 0x05
L_CCW = 0x03
L_STOP = 0x00
R_CW = 0x30
R_CCW = 0x50
R_STOP = 0x00

POW2_12 = 2**12
Kf = 0.2 # under 1.0
VEL_MAX = 1.0 # under 1.0
VEL_MIN = 0.01

#I2C = smbus.SMBus(1) #Jetson Nano
I2C = smbus.SMBus(8) #Jetson Xavier NX
I2C.write_byte_data(ADR_CTL,0x01, 0x00)

#EEPROM Write Initial Voltage : 0V
#I2C.write_byte_data(ADR_VEL_L,0x60, 0x00)
#I2C.write_byte_data(ADR_VEL_R,0x60, 0x00)

pub = rospy.Publisher('normal_vel', Twist, queue_size=10)

class main():
    def __init__(self):
        
        rospy.Subscriber('joy', Joy, self.call_joy_F710, queue_size=10)

        self.v_linear = 0.0
        self.v_angular = 0.0
        self.vln = 0.0
        self.vrn = 0.0

    def call_joy_F710(self,joy_msg): #F710_X
        nvel = Twist()
        if joy_msg.buttons[0] == 1:
            rospy.loginfo ('Push A/Green')
        elif joy_msg.buttons[1] == 1:
            rospy.loginfo ('Push B/Red')
        elif joy_msg.buttons[2] == 1:
            rospy.loginfo ('Push X/Blue')
        elif joy_msg.buttons[3] == 1:
            rospy.loginfo ('Push Y/Yellow')
        elif joy_msg.buttons[4] == 1:
            rospy.loginfo ('Push LB')
        elif joy_msg.buttons[5] == 1:
            rospy.loginfo ('Push RB')
        elif joy_msg.buttons[6] == 1:
            rospy.loginfo ('Push Back')
        elif joy_msg.buttons[7] == 1:
            rospy.loginfo ('Push Start')

        if joy_msg.axes[7] > 0.5:
            #rospy.loginfo ('Push Cross-Up')
            nvel.linear.x = 1.0
        elif joy_msg.axes[7] < -0.5:
            #rospy.loginfo ('Push Cross-Down')
            nvel.linear.x = -1.0
        else:
            nvel.linear.x = (joy_msg.axes[1] + joy_msg.axes[4]) / 2.0
        if joy_msg.axes[6] > 0.5:
            #rospy.loginfo ('Push Cross-Left')
            nvel.angular.z = 1.0
        elif joy_msg.axes[6] < -0.5:
            #rospy.loginfo ('Push Cross-Right')
            nvel.angular.z = -1.0
        else:
            nvel.angular.z = (joy_msg.axes[4] - joy_msg.axes[1]) / 2.0

        pub.publish(nvel)

        self.v_linear = nvel.linear.x
        self.v_angular = nvel.angular.z

    def move(self):

        self.vln = self.vln + Kf * (self.v_linear - self.v_angular - self.vln)
        self.vrn = self.vrn + Kf * (self.v_linear + self.v_angular - self.vrn)

        if self.vln > VEL_MAX:
            self.vln = VEL_MAX
        elif self.vln < -VEL_MAX:
            self.vln = -VEL_MAX
        if self.vrn > VEL_MAX:
            self.vrn = VEL_MAX
        elif self.vrn < -VEL_MAX:
            self.vrn = -VEL_MAX

	#Control Command / Left Wheel
        ctl_l = 0x00
        if self.vln > VEL_MIN:
            GPIO.output(L_SV,True)
            ctl_l = L_CW 
        elif self.vln < -VEL_MIN:
            GPIO.output(L_SV,True)
            ctl_l = L_CCW
        else:
            GPIO.output(L_SV,False)
            ctl_l = L_STOP

	#Control Command / Right Wheel
        ctl_r = 0x00
        if self.vrn > VEL_MIN:
            GPIO.output(R_SV,True)
            ctl_r = R_CW            
        elif self.vrn < -VEL_MIN:
            GPIO.output(R_SV,True)
            ctl_r = R_CCW
        else:
            GPIO.output(R_SV,False)
            ctl_r = R_STOP

        I2C.write_byte_data(ADR_CTL,0x13, ctl_l + ctl_r)

        vld = int((POW2_12-1) * abs(self.vln))
        vrd = int((POW2_12-1) * abs(self.vrn))

        vl0 = vld >> 8
        vl1 = vld & 0x00FF
        vr0 = vrd >> 8
        vr1 = vrd & 0x00FF
        # print(vld, vl0, vl1, vrd, vr0, vr1)
        
	# Output to Motor Driver
        I2C.write_byte_data(ADR_VEL_L,vl0, vl1)
        time.sleep(0.01)
        I2C.write_byte_data(ADR_VEL_R,vr0, vr1)
        time.sleep(0.01)

if __name__ == '__main__':
    rospy.init_node('joycon_node',anonymous=True)
    rospy.loginfo('Start joycon_node')
    r = rospy.Rate(20) #20Hz
    m = main()
    while not rospy.is_shutdown():
        m.move()
        r.sleep()
