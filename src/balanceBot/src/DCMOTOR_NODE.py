#!/usr/bin/env python

import rospy
import pigpio
from MotorAndEncoder.motor import motor
from std_msgs.msg import Float64

class dc_motor(object):
    def __init__(self):
        # Publishers and Subscribers
        self.u_sub = rospy.Subscriber('u', Float64, self.u_callback) # motor voltage command subscriber
        self.pos_pub = rospy.Publisher('current_position', Float64, queue_size=1) # position (in degrees) Publisher
        # Default Values
        self.u = 0
        self.u_prev = 0
        # Get Specific Parameters from ros parameter server
        pi = pigpio.pi()
        I1 = rospy.get_param('~I1')
        I2 = rospy.get_param('~I2')
        pwm = rospy.get_param('~pwm')
        encA = rospy.get_param('~encA')
        encB = rospy.get_param('~encB')
        # Initiate motor object (could have re-made this, but I'm re-using the old one)
        self.dcmotor = motor(pi,BI1,BI2,pwmB,enc2A,enc2B, encoder=True)
        # run the node
        self.runmotor()

    def runmotor(self):
        while not rospy.is_shutdown():
            if (self.u  != self.u_prev):
                self.dcmotor.set_duty_cycle(self.u)
            self.pos_pub.publish(self.dcmotor.get_pos())

    def u_callback(self, new_u_msg):
        self.u_prev = self.u
        self.u = new_u_msg.data

def dc_motor_node():
    rospy.init_node('motor')
    motor = dc_motor()

if __name__ == '__main__':
    try:
        dc_motor_node()
    except rospy.ROSInterruptException:
        pass
