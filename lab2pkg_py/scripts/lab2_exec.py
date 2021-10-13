#!/usr/bin/env python

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from math import pi
from lab2_header import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""
start = [175.87*pi/180.0, -80.43*pi/180.0, 90.02*pi/180.0,
-96.14*pi/180.0, -90.11*pi/180.0, 81.7*pi/180.0]

first_3 = [175.89*pi/180.0, -66.94*pi/180.0, 110.24*pi/180.0,
-129.85*pi/180.0, -90.14*pi/180.0, 81.61*pi/180.0]

first_2 = [175.9*pi/180.0, -60.72*pi/180.0, 112.67*pi/180.0,
-138.51*pi/180.0, -90.13*pi/180.0, 81.58*pi/180.0]

first_1 = [175.91*pi/180.0, -53.57*pi/180.0, 113.8*pi/180.0,
-146.78*pi/180.0, -90.12*pi/180.0, 81.56*pi/180.0]

second_up = [192.29*pi/180.0, -68.12*pi/180.0, 87.35*pi/180.0,
-105.97*pi/180.0, -89.13*pi/180.0, 98.07*pi/180.0]

second_3 = [192.31*pi/180.0, -58.99*pi/180.0, 97.64*pi/180.0,
-125.38*pi/180.0, -89.15*pi/180.0, 98.02*pi/180.0]

second_2 = [192.32*pi/180.0, -53.84*pi/180.0, 99.77*pi/180.0,
-132.65*pi/180.0, -89.14*pi/180.0, 98.01*pi/180.0]

second_1 = [192.33*pi/180.0, -48.5*pi/180.0, 100.73*pi/180.0,
-138.95*pi/180.0, -89.13*pi/180.0, 97.98*pi/180.0]

third_up = [206.73*pi/180.0, -54.79*pi/180.0, 62.37*pi/180.0,
-94.66*pi/180.0, -88.3*pi/180.0, 112.51*pi/180.0]

third_3 = [206.74*pi/180.0, -47.45*pi/180.0, 76.16*pi/180.0,
-115.81*pi/180.0, -88.33*pi/180.0, 112.46*pi/180.0]

third_2 = [206.75*pi/180.0, -43.45*pi/180.0, 78.36*pi/180.0,
-121.99*pi/180.0, -88.33*pi/180.0, 112.44*pi/180.0]

third_1 = [206.76*pi/180.0, -38.35*pi/180.0, 79.47*pi/180.0,
-128.2*pi/180.0, -88.32*pi/180.0, 112.43*pi/180.0]


Q = [ [start, first_1, first_2, first_3], [second_up, second_1, second_2, second_3], [third_up, third_1, third_2, third_3] ]

############### Your Code End Here ###############

############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""

def gripper_input_callback(msg):

    global gripper_current

    if(msg.DIGIN == False):
        gripper_current = False
    else:
        gripper_current = True 

############### Your Code End Here ###############


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


############## Your Code Start Here ##############
def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height):
    global Q

    ### Hint: Use the Q array to map out your towers by location and "height".

    error = 0

    move_arm(pub_cmd, loop_rate, Q[start_loc-1][0], 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, Q[start_loc-1][start_height], 4.0, 4.0)
    gripper(pub_cmd, loop_rate, suction_on)
    time.sleep(1.0)
    if (gripper_current==True):
        move_arm(pub_cmd, loop_rate, Q[start_loc-1][0], 4.0, 4.0)
        move_arm(pub_cmd, loop_rate, Q[end_loc-1][0], 4.0, 4.0)
        move_arm(pub_cmd, loop_rate, Q[end_loc-1][end_height], 4.0, 4.0)
        gripper(pub_cmd, loop_rate, suction_off)
        time.sleep(1.0)
        move_arm(pub_cmd, loop_rate, Q[end_loc-1][0], 4.0, 4.0)
    else:
        gripper(pub_cmd, loop_rate, suction_off)
        time.sleep(1.0)
        move_arm(pub_cmd, loop_rate, Q[0][0], 4.0, 4.0)
        print('The block has not been detected, please try again.')
        sys.exit()
    return error
############### Your Code End Here ###############


def main():

    global home
    global Q
    global SPIN_RATE

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message 
    # and corresponding callback function
    sub_gripper = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_input_callback)

    ############### Your Code End Here ###############


    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input

    start_position = 0
    end_position = 0
    input_done = 0

    while(not input_done):
        input_string = raw_input("Enter number of loops <Either 12 13 23 21 32 31 or 0 to quit> ")
        print("You entered " + input_string + "\n")

        if(int(input_string) == 12):
            start_position = 1
            end_position = 2
            input_done = 1
        elif (int(input_string) == 13):
            start_position = 1
            end_position = 3
            input_done = 1
        elif (int(input_string) == 23):
            start_position = 2
            end_position = 3
            input_done = 1
        elif (int(input_string) == 21):
            start_position = 2
            end_position = 1
            input_done = 1
        elif (int(input_string) == 31):
            start_position = 3
            end_position = 1
            input_done = 1
        elif (int(input_string) == 32):
            start_position = 3
            end_position = 2
            input_done = 1
        elif (int(input_string) == 0):
            print("Quitting... ")
            sys.exit()
        else:
            print("Please just enter the character 12 13 23 21 32 31 or 0 to quit \n\n")



    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input


    if(start_position>0):
        move_arm(pub_command, loop_rate, Q[start_position-1][0], 4.0, 4.0)
       
        medium_position = 6 - start_position - end_position

        move_block(pub_command, loop_rate, start_position, 3, end_position, 1)
        move_block(pub_command, loop_rate, start_position, 2, medium_position, 1)
        move_block(pub_command, loop_rate, end_position, 1, medium_position, 2)
        move_block(pub_command, loop_rate, start_position, 1, end_position, 1)
        move_block(pub_command, loop_rate, medium_position, 2, start_position, 1)
        move_block(pub_command, loop_rate, medium_position, 1, end_position, 2)
        move_block(pub_command, loop_rate, start_position, 1, end_position, 3)

        move_arm(pub_command, loop_rate, Q[end_position-1][0], 4.0, 4.0)


    ############### Your Code End Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass