import time
import sys
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)

GPIO.setup(13, GPIO.OUT)
GPIO.setup(15, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)

p27 = GPIO.PWM(13, 2000)  # channel=12 frequency=50Hz
p27.start(0)
p22 = GPIO.PWM(15, 2000)  # channel=12 frequency=50Hz
p22.start(0)
p10 = GPIO.PWM(19, 2000)  # channel=12 frequency=50Hz
p10.start(0)
p09 = GPIO.PWM(21, 2000)  # channel=12 frequency=50Hz
p09.start(0)

MAX_ABS_MOTOR_VELOCITY = 0.3

def set_tail_motor_velocity(v):
    if v < -MAX_ABS_MOTOR_VELOCITY:
        v = -MAX_ABS_MOTOR_VELOCITY
    elif v > MAX_ABS_MOTOR_VELOCITY:
        v = MAX_ABS_MOTOR_VELOCITY

    if v <= 0:
        p22.ChangeDutyCycle(0)
        p27.ChangeDutyCycle(-v * 100.0)
    else:
        p22.ChangeDutyCycle(v * 100.0)
        p27.ChangeDutyCycle(0)

def set_climb_motor_velocity(v):
    if v < -MAX_ABS_MOTOR_VELOCITY:
        v = -MAX_ABS_MOTOR_VELOCITY
    elif v > MAX_ABS_MOTOR_VELOCITY:
        v = MAX_ABS_MOTOR_VELOCITY

    if v <= 0:
        p10.ChangeDutyCycle(0)
        p09.ChangeDutyCycle(-v * 100.0)
    else:
        p10.ChangeDutyCycle(v * 100.0)
        p09.ChangeDutyCycle(0)


def schedule_tail_motor(v, t):
    try:
        set_tail_motor_velocity(v)
        time.sleep(t)
    finally:
        set_tail_motor_velocity(0)

def schedule_climb_motor(v, t):
    try:
        set_climb_motor_velocity(v)
        time.sleep(t)
    finally:
        set_climb_motor_velocity(0)


def forward(lv, lt, lw, rv, rt, rw, repeat):
    for i in range(repeat):
        schedule_tail_motor(lv, lt)
        schedule_tail_motor(0, lw)
        schedule_tail_motor(rv, rt)
        schedule_tail_motor(0, rw)

def dance():
    schedule_tail_motor(-0.2, 0.4)
    schedule_tail_motor(0, 0.2)
    schedule_tail_motor(-0.2, 0.4)
    schedule_tail_motor(0, 0.2)
    schedule_tail_motor(-0.2, 0.4)
    schedule_tail_motor(0, 0.2)
    schedule_tail_motor(-0.2, 0.4)
    schedule_tail_motor(0, 0.2)

    schedule_tail_motor(0.2, 0.4)
    schedule_tail_motor(0, 0.2)
    schedule_tail_motor(0.2, 0.4)
    schedule_tail_motor(0, 0.2)
    schedule_tail_motor(0.2, 0.4)
    schedule_tail_motor(0, 0.2)
    schedule_tail_motor(0.2, 0.4)
    schedule_tail_motor(0, 0.2)

    schedule_climb_motor(0.2, 1.2)
    schedule_climb_motor(-0.2, 1.2)

    schedule_climb_motor(0.2, 0.3)
    schedule_climb_motor(-0.2, 0.3)
    schedule_climb_motor(0.2, 0.3)
    schedule_climb_motor(-0.2, 0.3)


set_tail_motor_velocity(0)
set_climb_motor_velocity(0)
print ('cleanup gpio done')

