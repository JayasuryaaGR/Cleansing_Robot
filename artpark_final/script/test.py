#!/usr/bin/python3

import sys, rospy, time, actionlib, cv2
import moveit_commander, moveit_msgs.msg, geometry_msgs.msg
from sensor_msgs.msg import *
from std_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from math import pi
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.msg import LinkState 
from gazebo_msgs.srv import SetLinkState, GetModelState, DeleteModel


class MoveGroup(object):

    def __init__(self):
        super(MoveGroup, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self, j0, j1, j2, j3, j4, j5, j6):

        # print("From go_to_joint_state")

        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = j0
        joint_goal[1] = j1
        joint_goal[2] = j2
        joint_goal[3] = j3
        joint_goal[4] = j4
        joint_goal[5] = j5
        joint_goal[6] = j6
        move_group.go(joint_goal, wait=True)
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

def all_close(goal, actual, tolerance):

    # print("From All Close")

    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


def travel(tar_x, tar_y, tar_z, tar_w):

    # print("From Travel")

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = tar_x
    goal.target_pose.pose.position.y = tar_y
    goal.target_pose.pose.orientation.z = tar_z
    goal.target_pose.pose.orientation.w = tar_w

    client.send_goal(goal)
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


def water(model, link):

    # print("From Water")

    rospy.wait_for_service('/gazebo/get_model_state')
    get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    rospy.wait_for_service('/gazebo/set_link_state')
    set_state = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)

    robot_ini = get_state("artpark_robot", "world")
    state_msg = LinkState()
    model_state = get_state(model, "world")

    state_msg.link_name = link
    state_msg.pose.position.x = model_state.pose.position.x
    state_msg.pose.position.y = model_state.pose.position.y
    state_msg.pose.position.z = model_state.pose.position.z
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 0

    resp1 = set_state(state_msg)

    # print("Set State:")
    # print(resp1.success)
    # print(resp1.status_message)
    # print("\nResp2:")
    # print(resp2.link_state.pose.position.y)



def clean(model):
    global pub_move, move

    rospy.wait_for_service('/gazebo/delete_model')
    delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

    move.linear.x = 0.3
    pub_move.publish(move)
    time.sleep(6)
    move.linear.x = 0
    pub_move.publish(move)
    time.sleep(5)
    delete_model(model)
    time.sleep(2)
    move.linear.x = -0.3
    pub_move.publish(move)
    time.sleep(6)
    move.linear.x = 0
    pub_move.publish(move)
    # print("Clean Over")
    return



def camera_top(msg):

    global bridge

    def do_nothing(x):
        pass


    # Giving name to the window with Track Bars
    # And specifying that window is resizable
    cv2.namedWindow('Track Bars', cv2.WINDOW_NORMAL)

    # Defining Track Bars for convenient process of choosing colours
    # For minimum range
    cv2.createTrackbar('min_blue', 'Track Bars', 0, 255, do_nothing)
    cv2.createTrackbar('min_green', 'Track Bars', 0, 255, do_nothing)
    cv2.createTrackbar('min_red', 'Track Bars', 0, 255, do_nothing)

    # For maximum range
    cv2.createTrackbar('max_blue', 'Track Bars', 0, 255, do_nothing)
    cv2.createTrackbar('max_green', 'Track Bars', 0, 255, do_nothing)
    cv2.createTrackbar('max_red', 'Track Bars', 0, 255, do_nothing)

    # Reading image with OpenCV library
    # In this way image is opened already as numpy array
    # WARNING! OpenCV by default reads images in BGR format
    image_BGR = bridge.imgmsg_to_cv2(msg, "bgr8")
    # Resizing image in order to use smaller windows
    image_BGR = cv2.resize(image_BGR, (600, 426))

    # Showing Original Image
    # Giving name to the window with Original Image
    # And specifying that window is resizable
    cv2.namedWindow('Original Image', cv2.WINDOW_NORMAL)
    cv2.imshow('Original Image', image_BGR)

    # Converting Original Image to HSV
    image_HSV = cv2.cvtColor(image_BGR, cv2.COLOR_BGR2HSV)

    # Showing HSV Image
    # Giving name to the window with HSV Image
    # And specifying that window is resizable
    cv2.namedWindow('HSV Image', cv2.WINDOW_NORMAL)
    cv2.imshow('HSV Image', image_HSV)

    # Defining loop for choosing right Colours for the Mask
    while True:
        # Defining variables for saving values of the Track Bars
        # For minimum range
        min_blue = cv2.getTrackbarPos('min_blue', 'Track Bars')
        min_green = cv2.getTrackbarPos('min_green', 'Track Bars')
        min_red = cv2.getTrackbarPos('min_red', 'Track Bars')

        # For maximum range
        max_blue = cv2.getTrackbarPos('max_blue', 'Track Bars')
        max_green = cv2.getTrackbarPos('max_green', 'Track Bars')
        max_red = cv2.getTrackbarPos('max_red', 'Track Bars')

        # Implementing Mask with chosen colours from Track Bars to HSV Image
        # Defining lower bounds and upper bounds for thresholding
        mask = cv2.inRange(image_HSV,
                        (min_blue, min_green, min_red),
                        (max_blue, max_green, max_red))

        # Showing Binary Image with implemented Mask
        # Giving name to the window with Mask
        # And specifying that window is resizable
        cv2.namedWindow('Binary Image with Mask', cv2.WINDOW_NORMAL)
        cv2.imshow('Binary Image with Mask', mask)

        # Breaking the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    # Destroying all opened windows
    cv2.destroyAllWindows()


    # Printing final chosen Mask numbers
    print('min_blue, min_green, min_red = {0}, {1}, {2}'.format(min_blue, min_green,
                                                                min_red))
    print('max_blue, max_green, max_red = {0}, {1}, {2}'.format(max_blue, max_green,
                                                                max_red))


    """
    Some comments

    HSV (hue, saturation, value) colour-space is a model
    that is very useful in segmenting objects based on the colour.

    With OpenCV function cv2.inRange() we perform basic thresholding operation
    to detect an object based on the range of pixel values in the HSV colour-space.
    """



def camera_bottom(msg):

    global cam
    # print(cam)

    if cam:
        global y_min_blue, y_min_green, y_min_red, y_max_blue, y_max_green, y_max_red, yba
        global r_min_blue, r_min_green, r_min_red, r_max_blue, r_max_green, r_max_red, rba
        global p_min_blue, p_min_green, p_min_red, p_max_blue, p_max_green, p_max_red, pba
        global g_min_blue, g_min_green, g_min_red, g_max_blue, g_max_green, g_max_red, gba
        global panda, detected, detectedt, pub_move, move, bridge, r_detectedt, p_detectedt, g_detectedt, y_detectedt
        global r_detectedb, p_detectedb, g_detectedb, y_detectedb, then, now

        # print("From Cam Bot")

        image = bridge.imgmsg_to_cv2(msg, "bgr8")
        image_HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        y_mask = cv2.inRange(image_HSV, (y_min_blue, y_min_green, y_min_red), (y_max_blue, y_max_green, y_max_red))
        r_mask = cv2.inRange(image_HSV, (r_min_blue, r_min_green, r_min_red), (r_max_blue, r_max_green, r_max_red))
        p_mask = cv2.inRange(image_HSV, (p_min_blue, p_min_green, p_min_red), (p_max_blue, p_max_green, p_max_red))
        g_mask = cv2.inRange(image_HSV, (g_min_blue, g_min_green, g_min_red), (g_max_blue, g_max_green, g_max_red))

        v = cv2.__version__.split('.')[0]

        if v == '3':
            _, y_contours, _ = cv2.findContours(y_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            _, r_contours, _ = cv2.findContours(r_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            _, p_contours, _ = cv2.findContours(p_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            _, g_contours, _ = cv2.findContours(g_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        else:
            y_contours, _ = cv2.findContours(y_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            r_contours, _ = cv2.findContours(r_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            p_contours, _ = cv2.findContours(p_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            g_contours, _ = cv2.findContours(g_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        
        y_contours = sorted(y_contours, key=cv2.contourArea, reverse=True)
        r_contours = sorted(r_contours, key=cv2.contourArea, reverse=True)
        p_contours = sorted(p_contours, key=cv2.contourArea, reverse=True)
        g_contours = sorted(g_contours, key=cv2.contourArea, reverse=True)

        # print("Bot Contr: ")
        # print(r_contours)

        if y_contours and not detected and not y_detectedb:
            if cv2.contourArea(y_contours[0]) > yba:
                then = time.time()
                now = time.time()
                detected, y_detectedb = True, True
                move.linear.x = 0
                move.angular.z = 0
                pub_move.publish(move)
                time.sleep(1)
                print("From YD")
                panda.go_to_joint_state(-1.5708, 0.5, 0, -0.8, 0, 1.2, 0)
                water("Yellow_Mark", "yellow_mark_water")
                panda.go_to_joint_state(-1.5708, -0.5, 0, -0.8, 0, 0, 0)
                clean("Yellow_Mark")
                panda.go_to_joint_state(-1.5708, 0, 0, -0.8, 0, 0, 0)
                detected = False
                if y_detectedt or detectedt:
                    detectedt, y_detectedt = False, False
                back = True
                turn()
        
        elif r_contours and not detected and not r_detectedb:
            if cv2.contourArea(r_contours[0]) > rba:
                then = time.time()
                now = time.time()
                detected, r_detectedb = True, True
                move.linear.x = 0
                move.angular.z = 0
                pub_move.publish(move)
                time.sleep(1)
                # print(r_contours)
                # print(type(r_contours))
                # print("R Contour: " + str(cv2.contourArea(r_contours[0])))
                print("From RD")
                panda.go_to_joint_state(-1.5708, 0.5, 0, -0.8, 0, 1.2, 0)
                water("Red_Mark", "red_mark_water")
                panda.go_to_joint_state(-1.5708, -0.5, 0, -0.8, 0, 0, 0)
                clean("Red_Mark")
                panda.go_to_joint_state(-1.5708, 0, 0, -0.8, 0, 0, 0)
                detected = False
                if r_detectedt or detectedt:
                    detectedt, r_detectedt = False, False
                back = True
                turn()

        elif p_contours and not detected and not p_detectedb:
            if cv2.contourArea(p_contours[0]) > pba:
                then = time.time()
                now = time.time()
                detected, p_detectedb = True, True
                move.linear.x = 0
                move.angular.z = 0
                pub_move.publish(move)
                time.sleep(1)
                print("From PD")
                panda.go_to_joint_state(-1.5708, 0.5, 0, -0.8, 0, 1.2, 0)
                water("Purple_Mark", "purple_mark_water")
                panda.go_to_joint_state(-1.5708, -0.5, 0, -0.8, 0, 0, 0)
                clean("Purple_Mark")
                panda.go_to_joint_state(-1.5708, 0, 0, -0.8, 0, 0, 0)
                detected = False
                if p_detectedt or detectedt:
                    detectedt, p_detectedt = False, False
                # print("Calling Turn")
                back = True
                turn()

        elif g_contours and not detected and not g_detectedb:
            if cv2.contourArea(g_contours[0]) > gba:
                then = time.time()
                now = time.time()
                detected, g_detectedb = True, True
                move.linear.x = 0
                move.angular.z = 0
                pub_move.publish(move)
                time.sleep(1)
                print("From GD")
                panda.go_to_joint_state(-1.5708, 0.5, 0, -0.8, 0, 1.2, 0)
                water("Green_Mark", "green_mark_water")
                panda.go_to_joint_state(-1.5708, -0.5, 0, -0.8, 0, 0, 0)
                clean("Green_Mark")
                panda.go_to_joint_state(-1.5708, 0, 0, -0.8, 0, 0, 0)
                detected = False
                if g_detectedt or detectedt:
                    detectedt, g_detectedt = False, False
                back = True
                turn()
        
        cv2.waitKey(1)


def washbasin():
    global pub_move, move, panda, cam
    # print("From Washbasin")
    rospy.wait_for_service('/gazebo/set_link_state')
    set_state = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
    state_msg1 = LinkState()
    state_msg2 = LinkState()
    state_msg3 = LinkState()
    state_msg4 = LinkState()
    state_msg5 = LinkState()
    state_msg6 = LinkState()
    state_msg7 = LinkState()
    state_msg8 = LinkState()
    state_msg9 = LinkState()

    state_msg1.link_name = "washbasin_water_tr1"
    state_msg2.link_name = "washbasin_water_br1"
    state_msg3.link_name = "washbasin_water_tr2"
    state_msg4.link_name = "washbasin_water_br2"
    state_msg5.link_name = "washbasin_water_tr"
    state_msg6.link_name = "washbasin_water_br"
    state_msg7.link_name = "washbasin_water_inr"
    state_msg8.link_name = "washbasin_water_inl"
    state_msg9.link_name = "washbasin_water_inc"

    state_msg1.pose.position.x = -0.59
    state_msg1.pose.position.y = 0.68
    state_msg1.pose.position.z = 1.075
    state_msg1.pose.orientation.x = 0
    state_msg1.pose.orientation.y = 0
    state_msg1.pose.orientation.z = 0
    state_msg1.pose.orientation.w = 0

    state_msg2.pose.position.x = -0.59
    state_msg2.pose.position.y = 0.38
    state_msg2.pose.position.z = 1.075
    state_msg2.pose.orientation.x = 0
    state_msg2.pose.orientation.y = 0
    state_msg2.pose.orientation.z = 0
    state_msg2.pose.orientation.w = 0

    state_msg3.pose.position.x = -0.79
    state_msg3.pose.position.y = 0.68
    state_msg3.pose.position.z = 1.075
    state_msg3.pose.orientation.x = 0
    state_msg3.pose.orientation.y = 0
    state_msg3.pose.orientation.z = 0
    state_msg3.pose.orientation.w = 0

    state_msg4.pose.position.x = -0.79
    state_msg4.pose.position.y = 0.38
    state_msg4.pose.position.z = 1.075
    state_msg4.pose.orientation.x = 0
    state_msg4.pose.orientation.y = 0
    state_msg4.pose.orientation.z = 0
    state_msg4.pose.orientation.w = 0

    state_msg5.pose.position.x = -0.99
    state_msg5.pose.position.y = 0.68
    state_msg5.pose.position.z = 1.075
    state_msg5.pose.orientation.x = 0
    state_msg5.pose.orientation.y = 0
    state_msg5.pose.orientation.z = 0
    state_msg5.pose.orientation.w = 0

    state_msg6.pose.position.x = -0.99
    state_msg6.pose.position.y = 0.38
    state_msg6.pose.position.z = 1.075
    state_msg6.pose.orientation.x = 0
    state_msg6.pose.orientation.y = 0
    state_msg6.pose.orientation.z = 0
    state_msg6.pose.orientation.w = 0

    state_msg7.pose.position.x = -1.25
    state_msg7.pose.position.y = 0.5
    state_msg7.pose.position.z = 1.075
    state_msg7.pose.orientation.x = 0
    state_msg7.pose.orientation.y = 0
    state_msg7.pose.orientation.z = 0
    state_msg7.pose.orientation.w = 0

    state_msg8.pose.position.x = -1.52
    state_msg8.pose.position.y = 0.5
    state_msg8.pose.position.z = 1.075
    state_msg8.pose.orientation.x = 0
    state_msg8.pose.orientation.y = 0
    state_msg8.pose.orientation.z = 0
    state_msg8.pose.orientation.w = 0

    state_msg9.pose.position.x = -1.4
    state_msg9.pose.position.y = 0.45
    state_msg9.pose.position.z = 1.075
    state_msg9.pose.orientation.x = 0
    state_msg9.pose.orientation.y = 0
    state_msg9.pose.orientation.z = 0
    state_msg9.pose.orientation.w = 0

    time.sleep(2)
    panda.go_to_joint_state(-1.5708, 0, 0, -0.8, 0, 1.2, 0)

    resp1 = set_state(state_msg1)
    resp2 = set_state(state_msg2)
    resp3 = set_state(state_msg3)
    resp4 = set_state(state_msg4)
    resp5 = set_state(state_msg5)
    resp6 = set_state(state_msg6)

    # print("Set State:")
    # print(resp1.success)
    # print(resp1.status_message)

    move.angular.z = 0.5
    pub_move.publish(move)
    # print("Rotating...")
    time.sleep(7)
    move.angular.z = 0
    pub_move.publish(move)

    time.sleep(5)
    resp7 = set_state(state_msg7)
    resp8 = set_state(state_msg8)
    resp9 = set_state(state_msg9)

    panda.go_to_joint_state(-1.5708, 0, 0, -0.8, 0, 0, 0)

    # print("Set State:")
    # print(resp7.success)
    # print(resp7.status_message)

    # move.angular.z = 0.5
    # pub_move.publish(move)
    # time.sleep(6)

    cam = True
    # sub_camera = rospy.Subscriber("/camera_top/image_raw_top", Image, camera_top)
    # sub_camera = rospy.Subscriber("/camera_bottom/image_raw_bottom", Image, camera_bottom)

    return
    


def turn():
    global pub_move, move, robot_ini, detected, detectedt, cnt

    # now = time.time()
    # print("From Turn")
    detected, detectedt = False, False
    # while time.time() - now < 150:
        # print(detected)
    move.angular.z = 0.5
    # move.linear.x = -0.5
    pub_move.publish(move)
    time.sleep(1)

    if detected or detectedt:
        print("Detected")
        move.angular.z = 0
        pub_move.publish(move)
        time.sleep(1)
        cnt = 0
        # break

    # gap = time.time() - now
    # print("Gap:", gap)
    # if gap > 100:
    #     travel(robot_ini.pose.position.x, robot_ini.pose.position.y, robot_ini.pose.orientation.z, 1)

    return
    



if __name__ == '__main__':

    rospy.init_node('test')

    # pub_move = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    # move = Twist()

    # rospy.wait_for_service('/gazebo/get_model_state')
    # get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    # robot_ini = get_state("artpark_robot", "world")
    # state_msg = LinkState()

    # travel(-1, -1.2, 0.8, 1)
    # # print("Initial Travel Done")

    panda = MoveGroup()
    # detected, detectedt, r_detectedt, p_detectedt, g_detectedt, y_detectedt = False, False, False, False, False, False
    # r_detectedb, p_detectedb, g_detectedb, y_detectedb = False, False, False, False

    # move.linear.x = 0.5
    # pub_move.publish(move)
    # # print("Moving near WB")
    # time.sleep(5)
    # move.linear.x = 0
    # pub_move.publish(move)

    panda.go_to_joint_state(-1.5708, 0, 0, -0.8, 0, 0, 0)
    bridge = CvBridge()
    # bridget = CvBridge()

    # # Yellow Mark
    # y_min_blue, y_min_green, y_min_red = 0, 0, 255
    # y_max_blue, y_max_green, y_max_red = 45, 255, 255
    # yta, yba = 1000, 50000


    # # Red Mark
    # r_min_blue, r_min_green, r_min_red = 0, 0, 255
    # r_max_blue, r_max_green, r_max_red = 0, 255, 255
    # rta, rba = 5500, 55000

    # # Pink Mark
    # p_min_blue, p_min_green, p_min_red = 90, 0, 255
    # p_max_blue, p_max_green, p_max_red = 255, 255, 255
    # pta, pba = 650, 30000

    # # Green Mark
    # g_min_blue, g_min_green, g_min_red = 44, 0, 255
    # g_max_blue, g_max_green, g_max_red = 110, 255, 255
    # gta, gba = 1600, 65000

    # cam = False
    # cnt = 0

    # washbasin()

    # then = time.time()
    # now = time.time()

    # # print("Camera Started")
    # sub_camera = rospy.Subscriber("/camera_top/image_raw_top", Image, camera_top)
    sub_camera = rospy.Subscriber("/camera_bottom/image_raw_bottom", Image, camera_top)
    
    # turn()

    rospy.spin()

