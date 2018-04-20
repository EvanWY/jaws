#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

from time import sleep
from gpiozero import LED
from gpiozero import PWMOutputDevice
from gpiozero import OutputDevice
import threading

pitch_up_raw_signal = OutputDevice(4)
pitch_down_raw_signal = OutputDevice(17)
tail_left_raw_signal = OutputDevice(27)
tail_right_raw_signal = OutputDevice(22)

def pitch(val):
    if val > 0.5:
        pitch_up_raw_signal.off()
        pitch_down_raw_signal.on()
    elif val > -0.5:
        pitch_up_raw_signal.on()
        pitch_down_raw_signal.on()
    else:
        pitch_up_raw_signal.on()
        pitch_down_raw_signal.off()

def move_tail(val):
    if val > 0.5:
        tail_right_raw_signal.off()
        tail_left_raw_signal.on()
    elif val > -0.5:
        tail_right_raw_signal.on()
        tail_left_raw_signal.on()
    else:
        tail_right_raw_signal.on()
        tail_left_raw_signal.off()

pitch_rate = 0
yaw_rate = 0
speed = 0

def listener():
    global pitch_rate
    global yaw_rate
    global speed

    move_tail(0)
    pitch(0)
    rospy.init_node('jaws_movement_control_gateway')

    def pitch_rate_callback(data):
        global pitch_rate
        print (pitch_rate)
        pitch_rate=data.data
    rospy.Subscriber("/jaws/movement/pitch_rate", Float64, pitch_rate_callback)

    def yaw_rate_callback(data):
        global yaw_rate
        yaw_rate = data.data
    rospy.Subscriber("/jaws/movement/yaw_rate", Float64, yaw_rate_callback)

    def speed_callback(data):
        global speed
        speed = data.data
    rospy.Subscriber("/jaws/movement/speed", Float64, speed_callback)
    
    def ros_spin_worker():
        rospy.spin()
    threading.Thread(target=ros_spin_worker).start()

    while True:
        sleep(0)
        # pitch control
        pitch(pitch_rate)
        # speed and yaw rate control


if __name__ == '__main__':
    listener()
