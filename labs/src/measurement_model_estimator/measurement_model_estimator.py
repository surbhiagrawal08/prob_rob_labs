#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

feature_mean = None
threshold = 452
open = 0
closed = 0
pub_closed = None
pub_open = None

def feature_mean_callback(msg):
    global feature_mean, open, closed
    feature_mean = msg.data
    if feature_mean>=threshold:
        closed+=1
    else:
        open+=1
    return

def counter_publisher(event):
    global pub_closed, pub_open, closed, open

    pub_closed.publish(Float64(closed))
    pub_open.publish(Float64(open))
    #rospy.loginfo('im here')
    pass


def main():
    global pub_closed, pub_open
    rospy.init_node('measurement_model_estimator')
    rospy.loginfo('starting measurement_model_estimator')    
    pub_closed = rospy.Publisher('/closed_door_counter', Float64, queue_size=10)
    pub_open = rospy.Publisher('/open_door_counter', Float64, queue_size=10)
    rospy.Subscriber('/feature_mean', Float64, feature_mean_callback)
    #counter_publisher()
    rospy.Timer(rospy.Duration(0.1), counter_publisher)
    rospy.loginfo('done')
    rospy.spin()

if __name__=='__main__':
    main()
