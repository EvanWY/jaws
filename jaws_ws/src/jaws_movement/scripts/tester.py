#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

pub=None

def cb(data):
    global pub
    pub.publish(data.data+1)
    
def listener():
    global pub
    pub = rospy.Publisher('/jaws/movement/pitch_motor', Float64, queue_size=10)
    rospy.init_node('tester')
    rospy.Subscriber("/jaws/movement/tester", Float64, cb)
    rospy.spin()

if __name__ == '__main__':
    listener()