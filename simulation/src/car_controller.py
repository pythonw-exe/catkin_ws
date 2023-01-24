#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from simulation.msg import car_cmd

import sys
import curses

import numpy as np
from math import radians

# Joystick mapping
#Buttons
BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3
BUTTON_LB = 4
BUTTON_RB = 5
BUTTON_SELECT = 6
BUTTON_START = 7
BUTTON_HOME = 8
BUTTON_LJOY = 9
BUTTON_RJOY = 10

#Axis
AXES_LJOY_HOR = 0
AXES_LJOY_VER = 1
AXES_LTRIG = 2
AXES_RJOY_HOR = 3
AXES_RJOY_VER = 4
AXES_RTRIG = 5
AXES_DPAD_HOR = 6
AXES_DPAD_VER = 7

#temp
v_desired = 0
v = 0
dt = 0.1

#
class controller_class():
    def __init__(self, waypoints):
        #PID parameters
        self.K_P = 1
        self.K_D = 0.001
        self.K_I = 0.3

    def update_control(self):
        #Longtitudinal PID controller
        self.e = v_desired - v
        self.e_buffer.append(self.e)

        if len(self.e_buffer) >= 2:
            de = (self.e_buffer[1] - self.e_buffer[-2])/dt
            ie = sum(self.e_buffer)*dt
        else:
            de = 0.0
            ie = 0.0
        
        self.throttle = np.clip((self.K_P * self.e)+(self.K_D *de/dt) + (self.K_I*ie*dt), -1.0, 1.0)

#Human control
class keyboard_class():
    def __init__(self):
        #Publish car_cmd data
        self.pub_car_cmd = rospy.Publisher("/simulation/car_cmd", car_cmd, queue_size=1)
        self.msg_car_cmd = car_cmd()

    def process_keyboard(self, stdscr):
        while True:
            keycode = stdscr.getch()
            
            if keycode == curses.KEY_UP:
                self.msg_car_cmd.throttle += 0.001
                print("[car_controller-keyboard] process_keyboard: KEY_UP")

            elif keycode == curses.KEY_DOWN:
                self.msg_car_cmd.throttle -= 0.001
                print("[car_controller-keyboard] process_keyboard: KEY_DOWN")

            elif keycode == curses.KEY_LEFT:
                self.msg_car_cmd.delta += 0.01
                print("[car_controller-keyboard] process_keyboard: KEY_LEFT")

            elif keycode == curses.KEY_RIGHT:
                self.msg_car_cmd.delta -= 0.01
                print("[car_controller-keyboard] process_keyboard: KEY_RIGHT")
            
            self.pub_car_cmd.publish(self.msg_car_cmd)
            print("[car_controller-keyboard] process_key-msg_car_cmd: ", self.msg_car_cmd)


#Human control
class joystick_class():
    def __init__(self):
        #Create sub for /joy
        self.sub_joy = rospy.Subscriber("/joy", Joy, self.process_joy)

        #Publish car_cmd data
        self.pub_car_cmd = rospy.Publisher("/simulation/car_cmd", car_cmd, queue_size=1)
        self.msg_car_cmd = car_cmd()

    def process_joy(self, msg_joy):
        if msg_joy.axes[AXES_DPAD_VER] == 1:
            self.msg_car_cmd.thrust += 0.1
        elif msg_joy.axes[AXES_DPAD_VER] == -1:
            self.msg_car_cmd.thrust -= 0.1

        if msg_joy.axes[AXES_DPAD_HOR] == 1:
            self.msg_car_cmd.delta += 0.1
        elif msg_joy.axes[AXES_DPAD_HOR] == -1:
            self.msg_car_cmd.delta -= 0.1

        self.pub_car_cmd.publish(self.msg_car_cmd)

                # Process safety commands
        if msg_joy.buttons[BUTTON_RB]:
            print("test")
        elif msg_joy.buttons[BUTTON_A]:
            print("test")
        elif msg_joy.buttons[BUTTON_B]:
            print("test")

        # Process reset_estimator commands
        if msg_joy.buttons[BUTTON_LB]:
            print("test")
            
        print("[car_controller-joystick] process_joy-msg_car_cmd: ", self.msg_car_cmd)

if __name__ == "__main__":
    rospy.init_node("car_controller",anonymous=False)
    #joystick = joystick_class()
    keyboard = keyboard_class()
    curses.wrapper(keyboard.process_keyboard)
    rospy.spin()
    