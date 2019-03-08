#!/usr/bin/env python

import rospy
import pigpio
from MotorAndEncoder.motor import motor
from IMU.MPU6050 import MPU6050
from std_msgs.msg import Float64

class imu_node(object):
    def __init__(self):
        # Publishers and Subscribers
        # self.u_sub = rospy.Subscriber('u', Float64, self.u_callback) # motor voltage command subscriber
        self.pos_pub = rospy.Publisher('current_position', Float64, queue_size=1) # position (in degrees) Publisher
        # Default Values
        self.gx = 0
        self.u_prev = 0
        # Get Specific Parameters from ros parameter server
        pi = pigpio.pi()
        mpu_vio = rospy.get_param('~mpu_vio')
        # Initiate motor object (could have re-made this, but I'm re-using the old one)
        self.mpu = MPU6050(0x68, mpu_vio)

    def run(self):
        while not rospy.is_shutdown():
            #make sensor_msgs/imu message
            gx, gy, gz, _, ax, ay, az = mpu.get_all_data()
            self.pos_pub.publish(imu_msg)

def run_node():
    rospy.init_node('imu')
    imu = imu_node()
    imu.run()
    
if __name__ == '__main__':
    try:
        run_node()
    except rospy.ROSInterruptException:
        pass
