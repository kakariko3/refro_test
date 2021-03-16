#!/usr/bin/env python
import rospy
import serial
import time
from threading import Thread
import threading
from rospy.core import loginfo,logerr
from math import sin,cos,pi
from tf import transformations, TransformListener
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from geometry_msgs.msg import Twist

WHEEL_D = 0.15 #[m]
WIDTH = 0.408  #[m]
GEAR_RATIO = 15
PULSE = 2048
PULSE_PER_ROUND = GEAR_RATIO * PULSE
PULSE_2POW15 = 2**15
PULSE_2POW16 = 2**16
VEL_MAX = 0.47 #[m/s]
Kf = 0.3

TTYACM_L = '/dev/ttyACM_L'
TTYACM_R = '/dev/ttyACM_R'

SERIAL_RATE = 192000 #19200

#BASE_FRAME_ID = '/base_link'
#ODOM_FRAME_ID = 'odom'

STX = 0x02
ETX = 0x03
SCU_NO = '0'
GET = SCU_NO + 'BFA101' #Old:BFA001
SET = SCU_NO + 'BFA102' #Old:BFA002
SET_POS         = chr(STX) + SET + '27'          # ID39 zero + '0000'
GET_VEL         = chr(STX) + GET + '29'          # ID41
#GET_POS         = chr(STX) + GET + '28'          # ID40
GET_POS         = chr(STX) + GET + '2E'          # ID46
SET_MODE_VEL    = chr(STX) + SET + '1F' + '02'   # ID31 : 2
SET_SV_ON       = chr(STX) + SET + '1E' + '0100' # ID31 : 1
SET_SV_ON_POS   = chr(STX) + SET + '1E' + '0140' # ID31 : 1
SET_SV_OFF      = chr(STX) + SET + '1E' + '0000' # ID31 : 0
SET_USBCHK      = chr(STX) + SCU_NO + '7F2108'   # '7F211B'
SET_SV_TIMEOUT  = chr(STX) + SET + '94' + '00'   # '7F211B'
SET_VEL_TARGET  = chr(STX) + SET + '25'
SET_VEL_ZERO    = chr(STX) + SET + '25' + '0000'

LEFT = 0
RIGHT = 1

class main():
    def __init__(self):
        # Subscriber
        rospy.Subscriber('joymode', Bool, self.joymode, queue_size=10)
        rospy.Subscriber('cmd_vel', Twist, self.automove, queue_size=10)
        rospy.Subscriber('joy_nvel', Twist, self.joymove, queue_size=10)

        # Publisher
        self.pub_enc = rospy.Publisher("encoder_round",Float64MultiArray, queue_size=10)

        # Set Serial Port
        self.ser_l = serial.Serial(TTYACM_L, SERIAL_RATE, timeout=0.1)
        time.sleep(0.5)
        self.ser_r = serial.Serial(TTYACM_R, SERIAL_RATE, timeout=0.1)
        time.sleep(0.5)

        # Set Joymode
        self.joymode = True

        # Set Velocity
        self.vl_cur, self.vr_cur = 0.0, 0.0
        self.vl_mps, self.vr_mps = 0.0, 0.0
        self.latest_time = rospy.Time.now() 

        # Param
        self.count = 0
        self.step = 1

        # Set Mode Velocity
        print('Set Mode Velocity')
        ret = self.setCheckSum(SET_MODE_VEL)
        self.ser_l.write(ret)
        self.ser_r.write(ret)

        # Set USB Check
        #ret = self.setCheckSum(SET_USBCHK)
        #ser_l.write(ret)
        #ser_r.write(ret)

        # Set SV TIMEOUT
        #ret = self.setCheckSum(SET_SV_TIMEOUT)
        #self.ser_l.write(ret)
        #self.ser_r.write(ret)

        # Set Servo On and Position Initialize
        print('Set Servo On and Position Initialize')
        ret = self.setCheckSum(SET_SV_ON_POS)
        self.ser_l.write(ret)
        self.ser_r.write(ret)
        self.pos_l_old, self.pos_r_old = 0,0
        self.pos_l_cur, self.pos_r_cur = 0,0
        self.rot_l,     self.rot_r     = 0.0,0.0

    def joymode(self,msg):
        self.joymode = msg.data

    def automove(self,msg):
        if not self.joymode:
            self.vl_cur = msg.linear.x - msg.angular.z * WIDTH / 2
            self.vr_cur = msg.linear.x + msg.angular.z * WIDTH / 2         

    def joymove(self,msg):
        if self.joymode:
            self.vl_cur = VEL_MAX * (msg.linear.x - msg.angular.z)
            self.vr_cur = VEL_MAX * (msg.linear.x + msg.angular.z)

    def get_ascii_code(self,num):
        if num < 10:
            num += 0x30
        else:
            num += 0x37
        return num

    def setRpm(self, rpm, request):
        rpm1 = self.get_ascii_code((0x000F) & rpm)
        rpm2 = self.get_ascii_code(((0x00F0) & rpm) >> 4)
        rpm3 = self.get_ascii_code(((0x0F00) & rpm) >> 8)
        rpm4 = self.get_ascii_code(((0xF000) & rpm) >> 12)
        request += chr(rpm2)
        request += chr(rpm1)
        request += chr(rpm4)
        request += chr(rpm3)
        return request

    def setCheckSum(self, request):
        sumall = 0
        index = 0
        for char in request:
            if index == 0:
                index += 1
                continue
            sumall += ord(char)
            index += 1
        sum1 = self.get_ascii_code((0x000F) & sumall)
        sum2 = self.get_ascii_code(((0x00F0) & sumall) >> 4)
        sum3 = self.get_ascii_code(((0x0F00) & sumall) >> 8)
        sum4 = self.get_ascii_code(((0xF000) & sumall) >> 12)
        request += chr(int(sum4))
        request += chr(int(sum3))
        request += chr(int(sum2))
        request += chr(int(sum1))
        request += chr(ETX)
        return request

    def check_response(self, response,length):
        if response is None:
            logerr('None Response Error')
            return False
        if len(response) != length:
            logerr('Invalid Response : Length')
            return False
        if ord(response[0]) != STX:
            logerr('Invalid Response : No STX')
            return False
        if ord(response[length-1]) != ETX:
            logerr('Invalid Response : No ETX')
            return False             
        if ord(response[1]) != SCU_NO:
            logerr('Invalid Response : SCU_NO')
            return False
        return True

    def get_response(self, lr):
        pos_l, pos_r = 0,0
        readflag = False
        res = ''
        while(1):
            c = None
            if lr == LEFT:
                c = self.ser_l.read()
            else:
                c = self.ser_r.read()
            if c != None:
                if ord(c) == STX:
                    readflag = True
                    continue
                elif ord(c) == ETX:
                    break
                elif readflag:
                    res += c
        return res

    def get_rotnum(self, rot, pos_cur,pos_old):
        pos_d = pos_cur - pos_old
        if pos_d > PULSE_2POW15:
            rot += (pos_d - PULSE_2POW16) / float(PULSE_PER_ROUND)
        elif pos_d < -PULSE_2POW15:
            rot += (pos_d + PULSE_2POW16) / float(PULSE_PER_ROUND)
        else:
            rot += pos_d / float(PULSE_PER_ROUND)
        return rot

    def close(self):
        self.ser_l.close()
        self.ser_r.close()
        print('Serial Closed')

    def move(self):

        self.count += self.step
        if self.count > 100:
            self.step = -1
        elif self.count < -100:
            self.step = 1

        # m/s to rpm ------------------------------------------------
        self.vl_mps = self.vl_mps + Kf * (self.vl_cur - self.vl_mps)
        self.vr_mps = self.vr_mps + Kf * (self.vr_cur - self.vr_mps)
        if self.vl_mps > VEL_MAX:
            self.vl_mps = VEL_MAX
        elif self.vl_mps < -VEL_MAX:
            self.vl_mps = -VEL_MAX
        if self.vr_mps > VEL_MAX:
            self.vr_mps = VEL_MAX
        elif self.vr_mps < -VEL_MAX:
            self.vr_mps = -VEL_MAX

        left_rpm = int((self.vl_mps / WHEEL_D * pi) * 60) #180
        right_rpm  = int((-self.vr_mps / WHEEL_D * pi) * 60) #180

        ret = self.setRpm(left_rpm, SET_VEL_TARGET)
        ret = self.setCheckSum(ret)
        self.ser_l.write(ret)
        #self.ser_l.flush()
        ret_l = self.get_response(LEFT)

        ret = self.setRpm(right_rpm, SET_VEL_TARGET)
        ret = self.setCheckSum(ret)
        self.ser_r.write(ret)
        #self.ser_r.flush()
        ret_r = self.get_response(RIGHT)

        # Set USB Check
        #ret = self.setCheckSum(SET_USBCHK)
        #self.ser_l.write(ret)
        #self.ser_r.write(ret)

        #Get Encoder Position ---------------------------------------
        msg_enc = Float64MultiArray()

        ret_pos = self.setCheckSum(GET_POS)
        #Left Position
        self.ser_l.write(ret_pos)
        #self.ser_l.flush()
        ret = self.get_response(LEFT)
        if len(ret) > 13:
            self.pos_l_cur = int(ret[11]+ret[12],16) * 256 + int(ret[9]+ret[10],16)
        #Right Position
        self.ser_r.write(ret_pos)
        #self.ser_r.flush()
        ret = self.get_response(RIGHT)
        if len(ret) > 13:
            self.pos_r_cur = -int(ret[11]+ret[12],16) * 256 + int(ret[9]+ret[10],16)

        self.rot_l = self.get_rotnum(self.rot_l, self.pos_l_cur, self.pos_l_old)
        self.rot_r = self.get_rotnum(self.rot_r, self.pos_r_cur, self.pos_r_old)
        msg_enc.data = self.rot_l, self.rot_r
        self.pub_enc.publish(msg_enc)

        #print(self.count, left_rpm, ret_l, right_rpm, ret_r)
        self.pos_l_old = self.pos_l_cur
        self.pos_r_old = self.pos_r_cur

if __name__ == '__main__':
    rospy.init_node('move_node',anonymous=True)
    rospy.loginfo('Start move_node')
    r = rospy.Rate(20) #10Hz
    m = main()
    while not rospy.is_shutdown():
        m.move()
        r.sleep()
    m.close()
    time.sleep(1.0)
