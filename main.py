#! /usr/bin/env python

# launch empty world command: roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

import random
import time

import math

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from enum import Enum

from tf.transformations import euler_from_quaternion

x_robot = 0
y_robot = 0
yaw = 0
x_target = 4.0
y_target = 4.0
value_right = 0
value_front = 0
value_left = 0
value_halfleft = 0
value_halfright = 0
range_threshold = 0.4
speed_forward = 0.3
speed_rotating = 20  # degrees/sec
cornerDetected = False
is_stop_moving = False
vel_msg = Twist()


class RotationDirection(Enum):
    UNKNOWN = 0
    LEFT = 1
    RIGHT = 2


rotationDirection = RotationDirection.UNKNOWN


class RobotState(Enum):
    DRIVING = 0
    STOPPED = 1
    ROTATING = 2
    FOLLOW_WALL = 3


actualState = RobotState.STOPPED


def scan_callback(msg):  # reacting to new msgs from simulation
    global value_right, value_front, value_left, value_halfleft, value_halfright, cornerDetected
    ranges = msg.ranges

    front_range = ranges[349:359] + ranges[0:10]
    left_range = ranges[80:100]  # 90° --> right
    right_range = ranges[260:280]  # 270° --> left

    halfright_range = ranges[335:345]  # 340°
    halfleft_range = ranges[15:25]  # 20°

    value_left = min(left_range)
    value_right = min(right_range)
    value_front = min(front_range)

    value_halfright = min(halfright_range)
    value_halfleft = min(halfleft_range)


def pose_callback(pose_data):
    global x_robot, y_robot, yaw

    x_robot = pose_data.pose.pose.position.x
    y_robot = pose_data.pose.pose.position.y

    orientation_q = pose_data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)  # get yaw


def rotate_to_target():
    print(">>>>> Rotating to target direction")
    global x_target, y_target, yaw, vel_msg
    desired_angle_target = math.atan2(y_target - y_robot, x_target - x_robot)
    angular_speed = (desired_angle_target - yaw)
    print("angular_speed: " + str(angular_speed) + ", radians_speed: " + str(math.radians(speed_rotating)))
    while not (IsInDirection()):
        if (angular_speed < 0):
            rotate(-abs(math.radians(speed_rotating)))
        else:
            rotate(abs(math.radians(speed_rotating)))
    print(">>>>> Target angle reached")
    rotate(0)


def IsInDirection():
    desired_angle_goal = math.atan2(y_target - y_robot, x_target - x_robot)
    if (abs(desired_angle_goal - yaw)) < 0.15:
        return True
    return False


def follow_wall():
    angle = get_wall_angle()
    print("get angle: " + str(angle))
    print("rotating...")
    current_yaw = yaw

    angular_speed = math.radians(speed_rotating)

    if rotationDirection is RotationDirection.RIGHT:
        angular_speed = -abs(angular_speed)
    else:
        angular_speed = abs(angular_speed)

    while (abs(yaw - current_yaw) + abs(angle)) < math.radians(90):
        rotate(angular_speed)

    # variant 2 with time calculation:
    # current_angle = 0
    # t0 = rospy.Time.now().to_sec()
    # while(current_angle < abs(angle)):
    #     rotate(angular_speed)
    #     t1 = rospy.Time.now().to_sec()
    #     current_angle = angular_speed * (t1 - t0)

    print("Angle reached...")
    rotate(0)
    print("go on...")

    # Move robot forward after each rotation to prevent it from getting stuck.
    if rotationDirection is RotationDirection.LEFT:  # right
        move_forward()
        print("gucke rechts")
        rospy.sleep(0.5)
        while value_right < 0.4:
            pass
        move(0, 0)

    else:  # left
        move_forward()
        print("gucke links")
        rospy.sleep(0.5)
        while value_left < 0.4:
            pass
        move(0, 0)


def is_target_reached():  # Check if target is reached.
    if (x_target - 0.5 < x_robot < x_target + 0.5) and (y_target - 0.5 < y_robot < y_target + 0.5):
        return True
    return False


def get_wall_angle():  # Get the wall angle
    global rotationDirection

    angle = 0
    gamma = 20  # 20°

    print("value_front: " + str(value_front))
    print("value_halfleft: " + str(value_halfleft))
    print("value_halfright: " + str(value_halfright))

    if (value_halfleft is float("inf") and value_halfright is float("inf")):
        # random rotate
        rotationDirection = random.randint(1, 2)
        angle = 80
        print("value_halfleft and value_halfright is inf")
    else:
        a = 0
        b = 0

        if (value_halfright < value_halfleft):
            a = value_halfleft
            b = value_front
            rotationDirection = RotationDirection.LEFT
            print(str(rotationDirection))
        elif (value_halfright > value_halfleft):
            a = value_halfright
            b = value_front
            rotationDirection = RotationDirection.RIGHT
            print(str(rotationDirection))

        if (a != 0 and b != 0):
            c = math.sqrt(a ** 2 + b ** 2 - 2 * a * b * math.cos(math.radians(gamma)))
            tmp222 = ((-0.5 * (a ** 2) + 0.5 * (b ** 2) + 0.5 * (c ** 2)) / (b * c))
            alpha = math.degrees(math.acos(tmp222))
            angle = 180 - alpha - gamma

    print("Angle: " + str(angle) + ", rad: " + str(math.radians(angle)))
    return math.radians(angle)


def move(x, y):
    vel_msg.linear.x = x
    vel_msg.linear.y = y
    velocity_publisher.publish(vel_msg)


def move_forward():
    move(speed_forward, 0)
    return


def rotate(z):
    vel_msg.angular.z = z
    velocity_publisher.publish(vel_msg)


def Reset():
    move(0, 0)
    rotate(0)


def IsObstacleInFront():
    if value_front < range_threshold:
        return True
    return False


def StateHandler_STOPPED():
    if IsObstacleInFront():
        SetState(RobotState.FOLLOW_WALL)
    if not IsObstacleInFront() and not IsInDirection():
        SetState(RobotState.ROTATING)
    elif not IsObstacleInFront() and IsInDirection():
        SetState(RobotState.DRIVING)
    elif is_target_reached():
        SetState(RobotState.STOPPED)


def StateHandler_DRIVING():
    global actualState
    while not IsObstacleInFront():
        move_forward()
        if not IsInDirection():
            SetState(RobotState.ROTATING)
            break
        if IsObstacleInFront():
            SetState(RobotState.FOLLOW_WALL)
            break
        if is_target_reached():
            SetState(RobotState.STOPPED)
            break

    SetState(RobotState.STOPPED)


def StateHandler_ROTATING():
    global actualState
    while not IsInDirection():
        rotate_to_target()
        if IsInDirection():
            SetState(RobotState.DRIVING)
            break
    SetState(RobotState.STOPPED)


def StateHandler_FOLLOW_WALL():
    global actualState
    while IsObstacleInFront():
        follow_wall()
        if not IsObstacleInFront():
            SetState(RobotState.ROTATING)
            break
    SetState(RobotState.STOPPED)


def SetState(state: RobotState):
    global actualState
    actualState = state
    print(">>>>> Changed state to: " + str(state))


if __name__ == '__main__':
    # Set ROS nodes
    rospy.init_node('scan_node', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.Subscriber("/odom", Odometry, pose_callback)
    cmd_vel_topic = "/cmd_vel"
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    time.sleep(2.0)

    while not rospy.is_shutdown():

        while (value_front) < 0.15:
            print(">>>>> Collision detected!")
            move(-0.2, 0)
            Reset()

        # Check if goal is reached
        if is_target_reached():
            print(">>>>> Goal is reached")
            break

        # state handler
        if actualState == RobotState.STOPPED:
            StateHandler_STOPPED()
        elif actualState == RobotState.DRIVING:
            StateHandler_DRIVING()
        elif actualState == RobotState.ROTATING:
            StateHandler_ROTATING()
        elif actualState == RobotState.FOLLOW_WALL:
            StateHandler_FOLLOW_WALL()
        else:
            pass

        # cleaning
        Reset()

    Reset()
