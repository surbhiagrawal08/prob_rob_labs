#!/usr/bin/env python

import rospy
import numpy as np
import math
from opencv_apps.msg import Point2DArrayStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import Float64MultiArray
class Correspondance_matcher():
    def __init__(self):
        self.landmark_color = rospy.get_param('landmark_color')
        pred_sub = Subscriber('/'+self.landmark_color +'/predicted_corners', Point2DArrayStamped)
        feature_sub = Subscriber('/goodfeature_'+ self.landmark_color + '/corners', Point2DArrayStamped)
        ats = ApproximateTimeSynchronizer([pred_sub, feature_sub], queue_size=10, slop=0.2)
        ats.registerCallback(self.correspondance_callback)
        self.ordered_goodfeature_pub = rospy.Publisher('/ordered_goodfeature_/'+self.landmark_color, Point2DArrayStamped, queue_size=10)
        self.error_pub = rospy.Publisher('/feature_matching_error', Float64MultiArray, queue_size=10)
        self.covariance_pub = rospy.Publisher('/covariance_matrix', Float64MultiArray, queue_size=10)

    def correspondance_callback(self, pred_sub, feature_sub):
        total_error = 0.0
        reordered = False
        #ordered_feature_vec = []
        if reordered == False:
            ordered_feature_vec = []
            for i in range(4):
                dist = []
                #ordered_feature_vec = []
                for j in range(4):
                    euc_dist = (pred_sub.points[i].x - feature_sub.points[j].x)**2 + (pred_sub.points[i].y - feature_sub.points[j].y)**2
                    dist.append(euc_dist)
                min_index = dist.index(min(dist))
                #rospy.loginfo(dist)
                ordered_feature_vec.append(feature_sub.points[min_index])
                if len(ordered_feature_vec)==4:
                    reordered = True

        rospy.loginfo(f"Updated feature_sub points: {len(feature_sub.points)} points")

        pred_vec = np.array([pred_sub.points[0].x, pred_sub.points[0].y, pred_sub.points[1].x, pred_sub.points[1].y,
                    pred_sub.points[2].x, pred_sub.points[2].y, pred_sub.points[3].x, pred_sub.points[3].y])

        pred_vec = pred_vec.reshape(-1,1)

        feature_vec = np.array([ordered_feature_vec[0].x, ordered_feature_vec[0].y, ordered_feature_vec[1].x, ordered_feature_vec[1].y,
                          ordered_feature_vec[2].x, ordered_feature_vec[2].y, ordered_feature_vec[3].x, ordered_feature_vec[3].y])

        feature_vec = feature_vec.reshape(-1,1)

        error = feature_vec-pred_vec
        all_points = np.vstack((pred_vec.flatten(), feature_vec.flatten()))  # Stack both arrays
        covariance_matrix = np.cov(all_points) 
        #print(feature_sub)

            # Publish the average error
        #print("error", error)
        msg_error  = Float64MultiArray()
        msg_error.data = error.flatten() #error
        self.error_pub.publish(msg_error)
        msg = Point2DArrayStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.points = ordered_feature_vec #feature_sub.points
        self.ordered_goodfeature_pub.publish(msg) 

        msg_covariance = Float64MultiArray()
        msg_covariance.data = covariance_matrix.flatten()  # Flatten the covariance matrix before publishing
        self.covariance_pub.publish(msg_covariance)
        #return covariance_matrix, error

def main():
    rospy.init_node('corrrespondance_matcher')
    Correspondance_matcher()
    rospy.loginfo('starting corrrespondance_matcher')
    rospy.spin()
    rospy.loginfo('done')

if __name__=='__main__':
    main()
