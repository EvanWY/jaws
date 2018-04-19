#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

pub=None
def pitch_motor_callback(data):
    global pub
    rospy.loginfo(rospy.get_caller_id() + "I heard %f", data.data)
    pub.publish(data.data)
    
def listener():
    global pub
    pub = rospy.Publisher('/jaws/movement/tester', Float64, queue_size=10)
    rospy.init_node('jaws_movement_control_gateway')
    rospy.Subscriber("/jaws/movement/pitch_motor", Float64, pitch_motor_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()