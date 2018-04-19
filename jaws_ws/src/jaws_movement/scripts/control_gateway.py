#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

from time import sleep
from gpiozero import LED
from gpiozero import PWMOutputDevice
from gpiozero import OutputDevice

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

# while True:
#     a.off()
#     sleep(0.3)
#     a.on()
#     sleep(2)

#     for i in range(10):
#         a.off()
#         sleep(0.05)
#         a.on()
#         sleep(0.01)

#     a.on()
#     sleep(2)

def listener():
    rospy.init_node('jaws_movement_control_gateway')

    pitch_rate = 0
    yaw_rate = 0
    speed = 0
    move_tail(0)
    pitch(0)

    def pitch_rate_callback(data):
        pitch_rate = data.data
        pitch(pitch_rate)
    rospy.Subscriber("/jaws/movement/pitch_rate", Float64, pitch_rate_callback)

    def yaw_rate_callback(data):
        yaw_rate = data.data
    rospy.Subscriber("/jaws/movement/yaw_rate", Float64, yaw_rate_callback)

    def speed_callback(data):
        speed = data.data
    rospy.Subscriber("/jaws/movement/speed", Float64, speed_callback)
    
    rospy.spin()
    while True:
        pitch(pitch_rate)

if __name__ == '__main__':
    listener()
