#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import *
from geometry_msgs.msg import Twist
import time
from math import pi

rospy.init_node("working")

# stop = 0
sonar_front_stop = None
sonar_front_left_stop = None
sonar_front_right_stop = None
sonar_left_stop = None
sonar_right_stop = None
sonar_rear_stop = None
left_turn = 'a'

def sonar_front(msg):
    global sonar_front_stop
    sonar_front_stop = msg.range < 0.25
    rospy.loginfo("Sonar Front Callback " + str(sonar_front_stop) + " " + str(msg.range))

def sonar_front_left(msg):
    global sonar_front_left_stop
    sonar_front_left_stop = msg.range < 0.2
    rospy.loginfo("Sonar Front Left Callback " + str(sonar_front_left_stop) + " " + str(msg.range))

def sonar_front_right(msg):
    global sonar_front_right_stop
    sonar_front_right_stop = msg.range < 0.2
    rospy.loginfo("Sonar Front Right Callback " + str(sonar_front_right_stop) + " " + str(msg.range))

def sonar_rear(msg):
    global sonar_rear_stop
    sonar_rear_stop = msg.range < 0.2
    rospy.loginfo("Sonar Rear Callback " + str(sonar_rear_stop) + " " + str(msg.range))

def sonar_right(msg):
    global sonar_right_stop
    sonar_right_stop = msg.range < 0.2
    rospy.loginfo("Sonar Right Callback " + str(sonar_right_stop) + " " + str(msg.range))

def sonar_left(msg):
    global sonar_left_stop, left_turn
    sonar_left_stop = msg.range < 0.2
    if left_turn == 'a' and msg.range > 0.35:
        left_turn = 'b'
    rospy.loginfo("Sonar Left Callback " + str(sonar_left_stop) + " " + str(msg.range))

# def camera(msg):
#     msg 

pub_move = rospy.Publisher("/artpark_robot/cmd_vel", Twist, queue_size=10)
pub_body = rospy.Publisher("/artpark_robot/body_joint_effort_controller/command", Float64, queue_size=10)
pub_front_left_caster = rospy.Publisher("/artpark_robot/front_left_wheel_caster_joint_effort_controller/command", Float64, queue_size=10)
pub_front_right_caster = rospy.Publisher("/artpark_robot/front_right_wheel_caster_joint_effort_controller/command", Float64, queue_size=10)
pub_rear_left_caster = rospy.Publisher("/artpark_robot/rear_left_wheel_caster_joint_effort_controller/command", Float64, queue_size=10)
pub_rear_right_caster = rospy.Publisher("/artpark_robot/rear_right_wheel_caster_joint_effort_controller/command", Float64, queue_size=10)

move = Twist()
body = Float64()
front_left_caster = Float64()
front_right_caster = Float64()
rear_left_caster = Float64()
rear_right_caster = Float64()

body.data = 0
front_left_caster.data = 0
front_right_caster.data = 0
rear_left_caster.data = 0
rear_right_caster.data = 0
move.linear.x = 0.0

sub_sonar_front = rospy.Subscriber("/sonar_front", Range, sonar_front)
sub_sonar_front_left = rospy.Subscriber("/sonar_front_left", Range, sonar_front_left)
sub_sonar_front_right = rospy.Subscriber("/sonar_front_right", Range, sonar_front_right)
sub_sonar_rear = rospy.Subscriber("/sonar_rear", Range, sonar_rear)
sub_sonar_right = rospy.Subscriber("/sonar_right", Range, sonar_right)
sub_sonar_left = rospy.Subscriber("/sonar_left", Range, sonar_left)

rospy.loginfo("Outside")

# hz = rospy.Rate(20)

while not rospy.is_shutdown():
    rospy.loginfo("Speed = " + str(move.linear.x))
    if left_turn == 'b':
        time.sleep(0.5)
        body.data += pi/2
        front_left_caster.data += pi/2
        front_right_caster.data += pi/2
        rear_left_caster.data += pi/2
        rear_right_caster.data += pi/2
        rospy.loginfo("Turn Left")
        left_turn = 'c'
    elif sonar_front_stop and move.linear.x > 0:
        move.linear.x = -0.3
        body.data -= pi/2
        front_left_caster.data -= pi/2
        front_right_caster.data -= pi/2
        rear_left_caster.data -= pi/2
        rear_right_caster.data -= pi/2
        rospy.loginfo("Front Stopped")
        left_turn = 'a'
    elif sonar_front_left_stop and move.linear.x > 0:
        move.linear.x = -0.3
        body.data -= pi/6
        front_left_caster.data -= pi/6
        front_right_caster.data -= pi/6
        rear_left_caster.data -= pi/6
        rear_right_caster.data -= pi/6
        rospy.loginfo("Front Left Stopped")
        left_turn = 'a'
    elif sonar_front_right_stop and move.linear.x > 0:
        move.linear.x = -0.3
        body.data += pi/6
        front_left_caster.data += pi/6
        front_right_caster.data += pi/6
        rear_left_caster.data += pi/6
        rear_right_caster.data += pi/6
        rospy.loginfo("Front Right Stopped")
        left_turn = 'a'
    elif sonar_left_stop and move.linear.x > 0:
        move.linear.x = -0.3
        body.data -= pi/6
        front_left_caster.data -= pi/6
        front_right_caster.data -= pi/6
        rear_left_caster.data -= pi/6
        rear_right_caster.data -= pi/6
        rospy.loginfo("Left Stopped")
        left_turn = 'a'
    elif sonar_right_stop and move.linear.x > 0:
        move.linear.x = -0.3
        body.data += pi/6
        front_left_caster.data += pi/6
        front_right_caster.data += pi/6
        rear_left_caster.data += pi/6
        rear_right_caster.data += pi/6
        rospy.loginfo("Right Stopped")
        left_turn = 'a'
    elif sonar_rear_stop and move.linear.x < 0:
        move.linear.x = 0.8
        rospy.loginfo("Rear Stopped")
        left_turn = 'a'

    pub_move.publish(move)

    if move.linear.x == -0.3:
        time.sleep(1)
        move.linear.x = 0
        pub_move.publish(move)
        rospy.loginfo("Stopped Moving")

    pub_body.publish(body)
    pub_front_left_caster.publish(front_left_caster)
    pub_front_right_caster.publish(front_right_caster)
    pub_rear_left_caster.publish(rear_left_caster)
    pub_rear_right_caster.publish(rear_right_caster)
    rospy.loginfo("Published Everything")

    if move.linear.x == 0:
        move.linear.x = 0.8
        rospy.loginfo("Started Moving")
        time.sleep(2)
        pub_move.publish(move)
    
    # hz.sleep()

