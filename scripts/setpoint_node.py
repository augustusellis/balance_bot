#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Bool

class setpoint_node(object):
    def __init__(self):
        self.setpoint =  rospy.get_param('~setpoint')
        self.hz =  rospy.get_param('~rate')
        self.r = rospy.Rate(self.hz)
        self.pub = rospy.Publisher('setpoint', Float64, queue_size=1) # setpoint (in degrees)
        self.pubenable = rospy.Publisher('pid_enable', Bool, queue_size=1) # setpoint (in degrees)
        
        
    def run(self):
        while not rospy.is_shutdown():
            self.pub.publish(0)
            self.pubenable.publish(True)
            self.r.sleep()

def run_node():
    rospy.init_node('imu')
    setpoint = setpoint_node()
    setpoint.run()
    
if __name__ == '__main__':
    try:
        run_node()
    except rospy.ROSInterruptException:
        pass
