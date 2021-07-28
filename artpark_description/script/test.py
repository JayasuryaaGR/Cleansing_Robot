#!/usr/bin/python3
import rospy
from sensor_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import Twist
import time
from math import pi

rospy.init_node("test")
# time.sleep(5)

sonar_front_stop = None
sonar_front_left_stop = None
sonar_front_right_stop = None
sonar_left_stop = None
sonar_right_stop = None
sonar_rear_stop = None

can_go = True
move_left = True
cam = True

def publish():
    global pub_move, pub_body, pub_front_left_caster, pub_front_right_caster, pub_rear_left_caster, pub_rear_right_caster
    global can_left, move, body, front_left_caster, front_right_caster, rear_left_caster, rear_right_caster
    
    rospy.loginfo("From Function Publish")

    if move.linear.x == -0.3:
        pub_move.publish(move)
        time.sleep(0.5)
        move.linear.x = 0
        pub_move.publish(move)
        rospy.loginfo("Stopped Moving")

    pub_body.publish(body)
    pub_front_left_caster.publish(front_left_caster)
    pub_front_right_caster.publish(front_right_caster)
    pub_rear_left_caster.publish(rear_left_caster)
    pub_rear_right_caster.publish(rear_right_caster)
    rospy.loginfo("Published Everything")
    time.sleep(2)

    if move.linear.x == 0:
        move.linear.x = 0.8
        pub_move.publish(move)
        rospy.loginfo("Started Moving")

def front():
    global move, body, front_left_caster, front_right_caster, rear_left_caster, rear_right_caster
    move.linear.x = -0.3
    body.data -= pi/2
    front_left_caster.data -= pi/2
    front_right_caster.data -= pi/2
    rear_left_caster.data -= pi/2
    rear_right_caster.data -= pi/2
    rospy.loginfo("From Function Front")

def left():
    global move, body, front_left_caster, front_right_caster, rear_left_caster, rear_right_caster
    move.linear.x = -0.3
    body.data -= pi/6
    front_left_caster.data -= pi/6
    front_right_caster.data -= pi/6
    rear_left_caster.data -= pi/6
    rear_right_caster.data -= pi/6
    rospy.loginfo("From Function Left")

def right():
    global move, body, front_left_caster, front_right_caster, rear_left_caster, rear_right_caster
    move.linear.x = -0.3
    body.data += pi/6
    front_left_caster.data += pi/6
    front_right_caster.data += pi/6
    rear_left_caster.data += pi/6
    rear_right_caster.data += pi/6
    rospy.loginfo("From Function Right")

def left_turn():
    global move, body, front_left_caster, front_right_caster, rear_left_caster, rear_right_caster
    move.linear.x = 0
    body.data += pi/2
    front_left_caster.data += pi/2
    front_right_caster.data += pi/2
    rear_left_caster.data += pi/2
    rear_right_caster.data += pi/2
    rospy.loginfo("From Function Left Turn")

def left_turn_back():
    global move, body, front_left_caster, front_right_caster, rear_left_caster, rear_right_caster
    move.linear.x = 0
    body.data -= pi/2
    front_left_caster.data -= pi/2
    front_right_caster.data -= pi/2
    rear_left_caster.data -= pi/2
    rear_right_caster.data -= pi/2
    rospy.loginfo("From Function Left Turn")

def sonar_front(msg):
    global can_go, move, move_left
    if msg.range < 0.25 and move.linear.x > 0:
        move_left, can_go = False, False
        rospy.loginfo("Sonar Front Callback " + str(msg.range))
        front()
        rospy.loginfo("Front Stopped")
        publish()
        move_left, can_go = True, True


def sonar_front_left(msg):
    global can_go, move, move_left
    if msg.range < 0.2 and move.linear.x > 0:
        move_left, can_go = False, False
        rospy.loginfo("Sonar Front Left Callback " + str(msg.range))
        left()
        rospy.loginfo("Front Left Stopped")
        publish()
        move_left, can_go = True, True

def sonar_front_right(msg):
    global can_go, move, move_left
    if msg.range < 0.2 and move.linear.x > 0:
        move_left, can_go = False, False
        rospy.loginfo("Sonar Front Right Callback " + str(msg.range))
        right()
        rospy.loginfo("Front Right Stopped")
        publish()
        move_left, can_go = True, True

def sonar_rear(msg):
    global can_go, move, move_left
    if msg.range < 0.2 and move.linear.x < 0:
        move_left, can_go = False, False
        rospy.loginfo("Sonar Rear Callback " + str(msg.range))
        print(type(msg.range))
        move.linear.x = 0.8
        pub_move.publish(move)
        rospy.loginfo("Rear Stopped")
        move_left, can_go = True, True

def sonar_right(msg):
    global can_go, move, move_left
    if msg.range < 0.2 and move.linear.x > 0:
        move_left, can_go = False, False
        rospy.loginfo("Sonar Right Callback " + str(msg.range))
        right()
        rospy.loginfo("Right Stopped")
        publish()
        move_left, can_go = True, True

def sonar_left(msg):
    global can_go, move, move_left
    if msg.range < 0.2 and move.linear.x > 0:
        move_left, can_go = False, False
        rospy.loginfo("Sonar Left Callback " + str(msg.range))
        left()
        rospy.loginfo("Left Stopped")
        publish()
        move_left, can_go = True, True
    elif msg.range > 0.3 and move_left:
        time.sleep(0.5)
        rospy.loginfo("Calling Turn-Left Function as " + str(msg.range))
        left_turn()
        publish()
        # left_turn_back()
        # publish()
        move_left = False

    # elif msg.range > 0.6 and can_left:
    #     can_go = False
    #     rospy.loginfo("Left Overload " + str(msg.range))
    #     print(type(msg.range))
    #     left_turn()
    #     rospy.loginfo("Turned Left")
    #     publish()
    #     can_go = True

def camera(msg):
    pass
    # image = list(msg.data)
    # print(len(image))
    # # array is in image variable
    

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

rospy.loginfo("From Outside")

time.sleep(1)

pub_body.publish(body)
pub_front_left_caster.publish(front_left_caster)
pub_front_right_caster.publish(front_right_caster)
pub_rear_left_caster.publish(rear_left_caster)
pub_rear_right_caster.publish(rear_right_caster)

time.sleep(2)
pub_move.publish(move)

rospy.loginfo("Initially Published All")

# sub_camera = rospy.Subscriber("/camera1/image_raw", Image, camera, queue_size = 10)
sub_sonar_left = rospy.Subscriber("/sonar_left", Range, sonar_left)
sub_sonar_front = rospy.Subscriber("/sonar_front", Range, sonar_front)
sub_sonar_front_left = rospy.Subscriber("/sonar_front_left", Range, sonar_front_left)
sub_sonar_front_right = rospy.Subscriber("/sonar_front_right", Range, sonar_front_right)
sub_sonar_right = rospy.Subscriber("/sonar_right", Range, sonar_right)
sub_sonar_rear = rospy.Subscriber("/sonar_rear", Range, sonar_rear)

rospy.spin()

