#!/usr/bin/env python
import rospy
import math
import time
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Quaternion
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray

IMU_ENABLE = True

diameter = rospy.get_param('wheel_diameter',0.15)
tread = rospy.get_param('wheel_tread',0.408)
print ('diameter, tread : ',diameter,tread)
#rospy.loginfo ("diameter %f, tread: %f",diameter,tread)

enc = Float64MultiArray()
def call_enc(msg):
    global enc
    enc = msg

imu = Imu()
def call_imu(msg):
    global imu
    imu = msg

def odom_publisher():

    # Subscriber
    enc_sub = rospy.Subscriber("encoder_round",Float64MultiArray,callback=call_enc)
    imu_sub = rospy.Subscriber("imu",Imu,callback=call_imu)

    # Publisher
    pub = rospy.Publisher("odom", Odometry, queue_size=50)

    r = rospy.Rate(20) #20Hz
    #time.sleep(5)

    odom_x = 0.0
    odom_y = 0.0
    odom_th = 0.0
    last_time = rospy.get_time()
    while(len(enc.data)<2):
        continue
    while(enc.data[0] == 0 or enc.data[1] == 0):
        continue
    enc_l_old = enc.data[0]
    enc_r_old = enc.data[1]
    
    print(enc_l_old,enc_r_old)
 
    odom_bc = tf.TransformBroadcaster()
    while not rospy.is_shutdown():

        cur_time = rospy.get_time()
        dt = cur_time - last_time

        #read encoder : wheel round value
        enc_l_cur = enc.data[0] # left round
        enc_r_cur = enc.data[1] # right round
        enc_ld = enc_l_cur - enc_l_old
        enc_rd = enc_r_cur - enc_r_old
        enc_l_old = enc_l_cur
        enc_r_old = enc_r_cur

        #calc linear distance and angle per dt
        dx = (enc_rd + enc_ld) * diameter * math.pi / 2.0


        imu_vec = (imu.orientation.w,imu.orientation.x,imu.orientation.y,imu.orientation.z)
        imu_vec2 = (imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w)
        odom_euler = tf.transformations.euler_from_quaternion(imu_vec)
        odom_th = -odom_euler[0]
        dth = (enc_rd - enc_ld) / tread * diameter * math.pi
        #odom_th += dth

        odom_x -= dx * math.cos(odom_th)
        odom_y -= dx * math.sin(odom_th)

        #odom_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, odom_th)
        #if IMU_ENABLE:
        #    odom_quat[3] = local_imu.orientation.w
        #    odom_quat[0] = local_imu.orientation.x
        #    odom_quat[1] = local_imu.orientation.y
        #    odom_quat[2] = local_imu.orientation.z
        #    if local_imu.orientation.w < 0:
        #        odom_th = - local_imu.orientation.z * math.pi
        #    else:
        #        odom_th = local_imu.orientation.z * math.pi  

        odom_bc.sendTransform((odom_x, odom_y, 0.0), imu_vec2, rospy.Time.now(), "base_link", "odom")
        
        # publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # set the position
        odom.pose.pose.position.x = odom_x
        odom.pose.pose.position.y = odom_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = imu.orientation #Quaternion(*odom_quat)

        # set the velocity
        odom.twist.twist.linear.x = dx / dt
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = dth / dt

        # publish the message
        pub.publish(odom)
        last_time = cur_time
        r.sleep()

if __name__ == '__main__':
    rospy.init_node('odom_node',anonymous=True)
    rospy.loginfo ('Start odom')
    try:
        odom_publisher()
    except rospy.ROSInterruptException:
        pass
