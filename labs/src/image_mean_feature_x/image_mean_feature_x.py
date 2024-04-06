#!/usr/bin/env python

import rospy


def main():
    rospy.init_node('image_mean_feature_x')
    rospy.loginfo('starting image_mean_feature_x')
    rospy.spin()
    rospy.loginfo('done')

if __name__=='__main__':
    main()
