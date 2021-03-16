#!/usr/bin/env python
import rospy
import serial
import time
from threading import Thread
import threading
from rospy.core import loginfo,logerr
from math import sin,cos,pi
from tf import transformations, TransformListener
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from geometry_msgs.msg import Twist
WHEEL_D = 0.15
WIDTH = 0.408
GEAR_RATIO = 15
PULSE = 1024
PULSE_PER_ROUND = GEAR_RATIO * 1024
PULSE_2POW15 = 2**15
PULSE_2POW16 = 2**16

INC_ST_ACC = 0.006
INC_RT_ACC = 0.006

TTYACM_L = '/dev/ttyACM_L'
TTYACM_R = '/dev/ttyACM_R'

SERIAL_RATE = 375000 #19200

BASE_FRAME_ID = '/base_link'
ODOM_FRAME_ID = 'odom'

STX = 0x02
ETX = 0x03
SCU_NO = '0'

GET = SCU_NO + 'BFA101' #BFA001
SET = SCU_NO + 'BFA102' #BFA002

SET_POS         = chr(STX) + SET + '27'          # ID39 zero + '0000'
GET_POS         = chr(STX) + GET + '28'          # ID40
GET_POS         = chr(STX) + GET + '2E'          # ID46
#GET_VEL         = chr(STX) + GET + '29'          # ID41

SET_MODE_VEL    = chr(STX) + SET + '1F' + '02'   # ID31 : 2
SET_SV_ON       = chr(STX) + SET + '1E' + '0100' # ID31 : 1
SET_SV_ON_POS   = chr(STX) + SET + '1E' + '0140' # ID31 : 1
SET_SV_OFF      = chr(STX) + SET + '1E' + '0000' # ID31 : 0
SET_USBCHK      = chr(STX) + SCU_NO + '7F211B'   #
SET_VEL_TARGET  = chr(STX) + SET + '25'
SET_VEL_ZERO    = chr(STX) + SET + '25' + '0000'

LEFT = 0
RIGHT = 1

def get_ascii_code(num):
    if num < 10:
        num += 0x30
    else:
        num += 0x37
    return num

def setRpm(rpm, request):

    rpm1 = get_ascii_code((0x000F) & rpm)
    rpm2 = get_ascii_code(((0x00F0) & rpm) >> 4)
    rpm3 = get_ascii_code(((0x0F00) & rpm) >> 8)
    rpm4 = get_ascii_code(((0xF000) & rpm) >> 12)

    request += chr(rpm2)
    request += chr(rpm1)
    request += chr(rpm4)
    request += chr(rpm3)

    return request

def setCheckSum(request):
    sumall = 0
    index = 0
    for char in request:
        if index == 0:
            index += 1
            continue
        sumall += ord(char)
        index += 1
    sum1 = get_ascii_code((0x000F) & sumall)
    sum2 = get_ascii_code(((0x00F0) & sumall) >> 4)
    sum3 = get_ascii_code(((0x0F00) & sumall) >> 8)
    sum4 = get_ascii_code(((0xF000) & sumall) >> 12)
    request += chr(int(sum4))
    request += chr(int(sum3))
    request += chr(int(sum2))
    request += chr(int(sum1))
    request += chr(ETX)
    return request

def check_response(response,length):
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

def get_resolver_pulse(lr):
    pos_l, pos_r = 0,0
    readflag = False
    res = ''
    while(1):
        c = None
        if lr == LEFT:
            c = ser_l.read()
        else:
            c = ser_r.read()
        if c != None:
            if ord(c) == STX:
                readflag = True
                continue
            elif ord(c) == ETX:
                break
            elif readflag:
                res += c
    return res

def get_rot(rot, pos_cur,pos_old):
    pos_d = pos_cur - pos_old
    if pos_d > PULSE_2POW15:
        rot += (pos_d - PULSE_2POW16) / float(PULSE_PER_ROUND)
    elif pos_d < -PULSE_2POW15:
        rot += (pos_d + PULSE_2POW16) / float(PULSE_PER_ROUND)
    else:
        rot += pos_d / float(PULSE_PER_ROUND)
    return rot

def vel_handler(msg):
    x = msg.linear.x
    z = msg.angular.z
    vel_l = x - z * width / 2
    vel_r = x + z * width / 2

if __name__ == '__main__':
    rospy.init_node('joy2vel_node',anonymous=True)
    rospy.loginfo('Start movetest')
    pub = rospy.Publisher("cmd_vel_test", Twist, queue_size=10)

    ser_l = serial.Serial(TTYACM_L, SERIAL_RATE, timeout=1.0)
    ser_r = serial.Serial(TTYACM_R, SERIAL_RATE, timeout=1.0)

    r = rospy.Rate(10) #10Hz

    count = 0
    step = 1  

    #Set Mode Velocity
    print('Set Mode Velocity')
    ret = setCheckSum(SET_MODE_VEL)
    ser_l.write(ret)
    ser_r.write(ret)

    #Set USB Check
    #ret = setCheckSum(SET_USBCHK)
    #ser_l.write(ret)
    #ser_r.write(ret)

    #Set Servo On and Position Initialize
    print('Set Servo On and Position Initialize')
    ret = setCheckSum(SET_SV_ON_POS)
    ser_l.write(ret)
    ser_r.write(ret)

    pos_l_old,pos_r_old = 0,0
    pos_l_cur,pos_r_cur = 0,0
    rot_l,rot_r = 0.0,0.0

    while not rospy.is_shutdown():

        #if count > 100:
        #    step = -1
        #elif count < -100:
        #    step = 1
        count += step
            
        # m/s to rpm
        left_rpm = 180 #int((left_speed_m_s / self.wheel_diameter * pi) * 60)
        right_rpm = -180 #int((right_speed_m_s * (-1) / self.wheel_diameter * pi) * 60)
        ret = setRpm(left_rpm, SET_VEL_TARGET)
        ret = setCheckSum(ret)
        ser_l.write(ret)
        ret = setRpm(right_rpm, SET_VEL_TARGET)
        ret = setCheckSum(ret)
        ser_r.write(ret)

        #Get Encoder Position       
        ret_pos = setCheckSum(GET_POS)
        #Left Position
        ser_l.write(ret_pos)
        ret = get_resolver_pulse(LEFT)
        if len(ret) > 13:
            #print(ret[11]+ret[12]+ret[9]+ret[10])
            pos_l_cur = int(ret[11]+ret[12],16) * 256 + int(ret[9]+ret[10],16)
        #Right Position
        ser_r.write(ret_pos)
        ret = get_resolver_pulse(RIGHT)
        if len(ret) > 13:
            pos_r_cur = int(ret[11]+ret[12],16) * 256 + int(ret[9]+ret[10],16)

        rot_l = get_rot(rot_l, pos_l_cur, pos_l_old)
        rot_r = get_rot(rot_r, pos_r_cur, pos_r_old)


        print(count,pos_l_cur,pos_r_cur,rot_l,rot_r)
        pos_l_old = pos_l_cur
        pos_r_old = pos_r_cur

        r.sleep()

    ret = setCheckSum(SET_SV_OFF)
    ser_l.write(ret)
    ser_r.write(ret)
    time.sleep(1.0)
    ser_l.close()
    ser_r.close()
