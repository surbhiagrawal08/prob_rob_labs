#!/usr/bin/env python

import rospy
import numpy as np
from tf import transformations
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import PoseStamped

class Odometry_Error_Tracker():
    def __init__(self):
        self.distance_pub = rospy.Publisher('/ekf_odom_position_error', Float64, queue_size=10)
        self.angle_error_pub = rospy.Publisher('/ekf_odom_angle_error', Float64, queue_size=10)

        self.jackal_distance_pub = rospy.Publisher('/jackal_odom_distance', Float64, queue_size=10)
        self.jackal_angle_error_pub = rospy.Publisher('/jackal_odom_angle_error', Float64, queue_size=10)

        self.filtered_distance_pub = rospy.Publisher('/filtered_odom_distance', Float64, queue_size=10)
        self.filtered_angle_error_pub = rospy.Publisher('/filtered_odom_angle_error', Float64, queue_size=10)

        ekf_odom_sub = Subscriber('/ekf_odom', Odometry)
        gt_sub = Subscriber('/jackal/ground_truth/pose', PoseStamped)
        jackal_odom_sub = Subscriber('jackal_velocity_controller/odom', Odometry)
        filtered_odom_sum = Subscriber('/odometry/filtered', Odometry)
        ats = ApproximateTimeSynchronizer([ekf_odom_sub, gt_sub, jackal_odom_sub, filtered_odom_sum], queue_size=10, slop=0.2)
        ats.registerCallback(self.error_callback)

    def error_callback(self, ekf_odom, gt_pose, jackal_odom, filtered_odom):
        odom_position = ekf_odom.pose.pose.position
        odom_orientation = ekf_odom.pose.pose.orientation
        odom_quat = np.array([odom_orientation.x, odom_orientation.y, odom_orientation.z, odom_orientation.w])

        jackal_odom_position = jackal_odom.pose.pose.position
        jackal_odom_orientation = jackal_odom.pose.pose.orientation
        jackal_odom_quat = np.array([jackal_odom_orientation.x, jackal_odom_orientation.y, jackal_odom_orientation.z, jackal_odom_orientation.w])

        filtered_odom_position = filtered_odom.pose.pose.position
        filtered_odom_orientation = filtered_odom.pose.pose.orientation
        filtered_odom_quat = np.array([filtered_odom_orientation.x, filtered_odom_orientation.y, filtered_odom_orientation.z, filtered_odom_orientation.w])


        gt_position = gt_pose.pose.position
        gt_orientation = gt_pose.pose.orientation
        gt_quat = np.array([gt_orientation.x, gt_orientation.y, gt_orientation.z, gt_orientation.w])

        displacement = np.array([odom_position.x - gt_position.x, odom_position.y - gt_position.y, 0])
        distance = Float64()
        distance.data = np.linalg.norm(displacement)

        jackal_displacement = np.array([jackal_odom_position.x - gt_position.x, jackal_odom_position.y - gt_position.y, 0])
        jackal_distance = Float64()
        jackal_distance.data = np.linalg.norm(jackal_displacement)

        filtered_displacement = np.array([filtered_odom_position.x - gt_position.x, filtered_odom_position.y - gt_position.y, 0])
        filtered_distance = Float64()
        filtered_distance.data = np.linalg.norm(filtered_displacement)

        odom_ori_euler = transformations.euler_from_quaternion(odom_quat, 'sxyz')
        gt_ori_euler = transformations.euler_from_quaternion(gt_quat, 'sxyz')
        jackal_ori_euler = transformations.euler_from_quaternion(jackal_odom_quat, 'sxyz')
        filtered_ori_euler = transformations.euler_from_quaternion(filtered_odom_quat, 'sxyz')

        angle_error = Float64()
        angle_error.data = np.abs(odom_ori_euler[2] - gt_ori_euler[2])

        jackal_angle_error = Float64()
        jackal_angle_error.data = np.abs(jackal_ori_euler[2] - gt_ori_euler[2])

        filtered_angle_error = Float64()
        filtered_angle_error.data = np.abs(filtered_ori_euler[2] - gt_ori_euler[2])

        self.distance_pub.publish(distance)
        self.angle_error_pub.publish(angle_error)
        self.jackal_distance_pub.publish(jackal_distance)
        self.jackal_angle_error_pub.publish(jackal_angle_error)
        self.filtered_distance_pub.publish(filtered_distance)
        self.filtered_angle_error_pub.publish(filtered_angle_error)


def main():
    rospy.init_node('odometry_error_tracker')
    rospy.loginfo('starting odometry_error_tracker')
    odometry_error_tracker = Odometry_Error_Tracker()
    rospy.spin()
    rospy.loginfo('done')

if __name__=='__main__':
    main()
