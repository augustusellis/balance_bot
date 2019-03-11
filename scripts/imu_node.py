#!/usr/bin/env python

import rospy
from MotorAndEncoder.motor import motor
from IMU.MPU6050 import MPU6050
from IMU.ComplementaryFilter import ComplementaryFilter
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu

class imu_node(object):
    def __init__(self):
        # Publishers and Subscribers
        # self.u_sub = rospy.Subscriber('u', Float64, self.u_callback) # motor voltage command subscriber
        self.pos_pub = rospy.Publisher('current_imu', Float64, queue_size=1) # position (in degrees) Publisher
        # Default Values
        #self.imu_msg = Imu()
        self.theta = 0
        # Get Specific Parameters from ros parameter server
        #self.pi = pigpio.pi()
        self.mpu_vio = rospy.get_param('~mpu_vio')
        # create filter
        self.cfilt = ComplementaryFilter(alpha=0.98, rollangle=0, angleOffset=0.730238683)
        # start timing
        self.current_time = rospy.Time.now().to_sec()
        self.previous_time = self.current_time
        # Initiate imu object
        self.mpu = MPU6050(0x68, self.mpu_vio)
        # rate object
        self.r = rospy.Rate(400)

    def run(self):
        while not rospy.is_shutdown():
            # timing
            self.previous_time = self.current_time
            self.current_time = rospy.Time.now().to_sec()
            #make sensor_msgs/imu message
            gx, gy, gz, _, ax, ay, az = self.mpu.get_all_data()
            self.cfilt.update([gx,gy,gz,ax,ay,az], (self.current_time-self.previous_time))
            #print(self.current_time - self.previous_time)
            #self.imu_msg.angular_velocity.x = gx
            #self.imu_msg.angular_velocity.y = gy
            #self.imu_msg.angular_velocity.z = gz
            #self.imu_msg.linear_acceleration.x = ax
            #self.imu_msg.linear_acceleration.y = ay
            #self.imu_msg.linear_acceleration.z = az
            self.pos_pub.publish(self.cfilt.rollangle)
            self.r.sleep()

def run_node():
    rospy.init_node('imu')
    imu = imu_node()
    imu.run()
    
if __name__ == '__main__':
    try:
        run_node()
    except rospy.ROSInterruptException:
        pass
