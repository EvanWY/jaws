#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

from time import sleep
import time
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

    stage = 0 # stop <--> l->break->r->r->break->l
    stage_start_time = time.time()
    base_duration = 0.12
    delta_duration = 0.08

    speed_swing_param = 0.85
    speed_pause_param = 2

    lastStage = stage
    cached_speed = speed
    while True:
        sleep(0)

        # pitch control
        pitch(pitch_rate)

        # speed and yaw rate control
        yaw_factor = delta_duration*yaw_rate*0.5
        if stage == 0:
            if speed > 0:
                #transit
                stage = 1
                stage_start_time = time.time()
                move_tail(-1)            
                #end transit
        elif stage == 1:
            stage_end_time = stage_start_time + (speed_swing_param * (1-speed) + speed) * (base_duration + yaw_factor+abs(yaw_factor))
            if time.time() >= stage_end_time:
                #transit
                stage = 2
                stage_start_time = time.time()
                move_tail(0)            
                #end transit
        elif stage == 2:
            stage_end_time = stage_start_time + 0.02 + (1-speed) * speed_pause_param
            if time.time() >= stage_end_time:
                #transit
                stage = 3
                stage_start_time = time.time()
                move_tail(1)            
                #end transit
        elif stage == 3:
            stage_end_time = stage_start_time + (speed_swing_param * (1-speed) + speed) * (base_duration + yaw_factor-abs(yaw_factor))
            if time.time() >= stage_end_time:
                if speed <= 0:
                    #transit
                    stage = 0
                    stage_start_time = time.time()
                    move_tail(0)            
                    #end transit
                else:
                    #transit
                    stage = 4
                    stage_start_time = time.time()
                    move_tail(1)            
                    #end transit
        elif stage == 4:
            stage_end_time = stage_start_time + (speed_swing_param * (1-speed) + speed) * (base_duration + yaw_factor-abs(yaw_factor))
            if time.time() >= stage_end_time:
                #transit
                stage = 5 
                stage_start_time = time.time()
                move_tail(0)            
                #end transit
        elif stage == 5:
            stage_end_time = stage_start_time + 0.02 + (1-speed) * speed_pause_param
            if time.time() >= stage_end_time:
                #transit
                stage = 6
                stage_start_time = time.time()
                move_tail(-1)            
                #end transit
        elif stage == 6:
            stage_end_time = stage_start_time + (speed_swing_param * (1-speed) + speed) * (base_duration + yaw_factor+abs(yaw_factor))
            if time.time() >= stage_end_time:
                if speed <= 0:
                    #transit
                    stage = 0
                    stage_start_time = time.time()
                    move_tail(0)            
                    #end transit
                else:
                    #transit
                    stage = 1
                    stage_start_time = time.time()
                    move_tail(-1)            
                    #end transit



if __name__ == '__main__':
    listener()
