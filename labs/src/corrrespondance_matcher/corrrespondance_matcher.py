#!/usr/bin/env python

import rospy
import numpy as np
import math
from opencv_apps.msg import Point2DArrayStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import Float64
class Correspondance_matcher():
    def __init__(self):
        self.landmark_color = rospy.get_param('landmark_color')
        pred_sub = Subscriber('/'+self.landmark_color +'/predicted_corners', Point2DArrayStamped)
        feature_sub = Subscriber('/goodfeature_'+ self.landmark_color + '/corners', Point2DArrayStamped)
        ats = ApproximateTimeSynchronizer([pred_sub, feature_sub], queue_size=10, slop=0.2)
        ats.registerCallback(self.correspondance_callback)
        self.ordered_goodfeature_pub = rospy.Publisher('/ordered_goodfeature_/'+self.landmark_color, Point2DArrayStamped, queue_size=10)
        self.error_pub = rospy.Publisher('/feature_matching_error', Float64, queue_size=10)

    def correspondance_callback(self, pred_sub, feature_sub):
        total_error = 0.0

        for i in range(4):
            dist = []
            for j in range(4):
                euc_dist = (pred_sub.points[i].x - feature_sub.points[j].x)**2 + (pred_sub.points[i].y - feature_sub.points[j].y)**2
                dist.append(euc_dist)
            min_index = dist.index(min(dist))
            feature_sub.points[i] = feature_sub.points[min_index]
        #rospy.loginfo(f"Updated feature_sub points: {len(feature_sub.points)} points")

            error = math.sqrt(dist[min_index])
            total_error += error
            rospy.loginfo(f"Error: {error}")
        avg_error = total_error / 4
            # Publish the average error
        msg_error  = Float64()
        msg_error.data = avg_error
        self.error_pub.publish(msg_error)
        msg = Point2DArrayStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.points = feature_sub.points
        self.ordered_goodfeature_pub.publish(msg)
        #rospy.loginfo("I m here")














def main():
    rospy.init_node('corrrespondance_matcher')
    Correspondance_matcher()
    rospy.loginfo('starting corrrespondance_matcher')
    rospy.spin()
    rospy.loginfo('done')

if __name__=='__main__':
    main()
