#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped, TwistStamped

class GroundTruthExtractor:
    def __init__(self):
        self.sub_gaz = rospy.Subscriber('/gazebo/link_states', LinkStates, self.ground_truth_callback,
                                        queue_size=10)
        self.pub_gt_twist = rospy.Publisher('/jackal/ground_truth/twist', TwistStamped, 
                                            queue_size = 10)
        self.pub_gt_pose = rospy.Publisher('/jackal/ground_truth/pose', PoseStamped,
                                           queue_size = 10)        

    def ground_truth_callback(self, link_states):
        self.link_names = link_states.name
        base_index = self.link_names.index("jackal::base_link")
        now = rospy.get_rostime()
        gt_twist = TwistStamped()
        gt_pose = PoseStamped()

        gt_twist.header.stamp, gt_pose.header.stamp = now, now
        gt_twist.header.frame_id = "base_link"
        gt_pose.header.frame_id = "odom"

        gt_twist.twist = link_states.twist[base_index]
        gt_pose.pose = link_states.pose[base_index]        

        self.pub_gt_pose.publish(gt_pose)
        self.pub_gt_twist.publish(gt_twist)
        
def main():
    rospy.init_node('ground_truth_extractor')
    rospy.loginfo('starting ground_truth_extractor')
    ground_truth_extractor = GroundTruthExtractor()
    rospy.spin()
    rospy.loginfo('done')

if __name__=='__main__':
    main()
