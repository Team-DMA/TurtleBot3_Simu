#! /usr/bin/env python

import time
import tf
import math

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

x_robot = 0
y_robot = 0
yaw = 0
x_target = 2.0
y_target = 1.0
value_right = 0
value_front = 0
value_left = 0
is_stop_moving = False
vel_msg = Twist()


def scan_callback(msg):  # reacting to new msg from simulation
    global value_right, value_front, value_left
    ranges = msg.ranges
    front_range = ranges[339:359] + ranges[0:20]
    right_range = ranges[75:95]
    left_range = ranges[255:275]

    value_left = min(left_range)
    value_right = min(right_range)
    value_front = min(front_range)


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
    K_angular = 0.5
    speedFactor = 100
    # angular_speed = (desired_angle_target - yaw) * K_angular
    target_rad = desired_angle_target * math.pi / 180 * speedFactor
    angular_speed = K_angular * (target_rad - yaw)
    # print("Yaw: " + str(yaw) + ", target: " + str(desired_angle_target))
    while not (IsInDirection()):
        rotate(angular_speed)
    print(">>>>> Target angle reached")
    # print("Target Angle reached: Yaw: " + str(yaw) + ", target: " + str(desired_angle_target))
    rotate(0)


def IsInDirection():
    desired_angle_goal = math.atan2(y_target - y_robot, x_target - x_robot)
    if (abs(desired_angle_goal - yaw)) < 0.1:
        return True
    return False


def move_forward():
    # print(">>>>> Move forward")
    vel_msg = Twist()
    vel_msg.linear.x = 0.3
    velocity_publisher.publish(vel_msg)
    return


def follow_wall():
    if value_right > value_left:
        angle = get_wall_angle(True)
        print(">>>>> Change angle to right (90 degrees): ", angle)
    else:
        angle = get_wall_angle(False)
        print(">>>>> Change angle to left (90 degrees): ", angle)
    if (angle - yaw) < 0:
        while (angle - yaw) < -0.1:
            vel_msg.angular.z = (angle - yaw) * 0.9
            velocity_publisher.publish(vel_msg)
    else:
        while (angle - yaw) > 0.1:
            vel_msg.angular.z = (angle - yaw) * 0.9
            velocity_publisher.publish(vel_msg)
    vel_msg.angular.z = 0
    # Move robot forward (for amount of time) after each rotation to prevent it from getting stuck.
    if value_front > 0.4:
        time = 0
        t0 = rospy.Time.now().to_sec()
        while time < 1.3 and value_front > 0.4:
            t1 = rospy.Time.now().to_sec()
            time = t1 - t0
            vel_msg.linear.x = 0.3
            velocity_publisher.publish(vel_msg)


def is_target_reached():  # Check if target is reached.
    if (x_target - 0.5 < x_robot < x_target + 0.5) and (y_target - 0.5 < y_robot < y_target + 0.5):
        return True
    return False


def get_wall_angle(is_right):  # Get the wall angle
    angle = 0
    if ((value_left > 0.6 and yaw > 0) or (value_right > 0.6 and yaw < 0)) and not (
            (0.0 < yaw < 0.002) or (-3.16 < yaw < -3.0)):
        angle = 0.0 if is_right else -math.pi
    else:
        angle = -math.pi / 2 if is_right else math.pi / 2
    return angle


def move(x, y):
    vel_msg.angular.z = x
    vel_msg.linear.x = y
    velocity_publisher.publish(vel_msg)
    if x == 0.0 and y == 0.0:
        SetMovingState(False)


def rotate(z):
    vel_msg.angular.z = z
    velocity_publisher.publish(vel_msg)


def SetMovingState(boolean):
    global is_stop_moving
    is_stop_moving = boolean


def GetMovingState():
    return is_stop_moving


def Reset():
    move(0, 0)
    rotate(0)


if __name__ == '__main__':
    # Set ROS nodes.
    rospy.init_node('scan_node', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.Subscriber("/odom", Odometry, pose_callback)
    cmd_vel_topic = "/cmd_vel"
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    time.sleep(2.0)

    while not rospy.is_shutdown():
        # Print data.
        # print(">>>>> Data: value_right: " + str(value_right) + ", value_left: " + str(value_left) + ", value_front: "
        #       + str(value_front) + ", yaw: " + str(yaw))

        # check dist from wall
        if value_front > 0.4:
            # If robot is not following wall...
            if not ((value_right < 0.4 and yaw < 0) or (value_left < 0.4 and yaw > 0)):
                rotate_to_target()
                SetMovingState(True)
            while value_front > 0.4:
                move_forward()
                # If robot is not following wall...
                if not ((value_right < 0.4 and yaw < 0) or (value_left < 0.4 and yaw > 0)):
                    # If robot is not in the direction of the goal...
                    if (not IsInDirection()) and GetMovingState():
                        print(">>>>> Stop moving forward")
                        move(0.0, 0.0)
                        break

        else:
            follow_wall()

        # Do some cleaning.
        Reset()

        # Check is goal reached.
        is_reached = is_target_reached()
        if is_reached:
            print(">>>>> Goal is reached")
            break

    Reset()
