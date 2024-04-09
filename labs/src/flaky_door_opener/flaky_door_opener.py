#!/usr/bin/env python

import rospy
import threading
import random

from std_msgs.msg import Empty
from std_msgs.msg import Float64

heartbeat_rate = 50
max_torque = 10

class DoorOpener():
    def __init__(self):
        self.lock = threading.Lock()
        self.open_sub = rospy.Subscriber(
            'door_open', Empty, self.handle_open_request,
            queue_size = 1)
        self.pub_torque = rospy.Publisher(
            '/hinged_glass_door/torque', Float64, queue_size = 1)

    def handle_open_request(self, _):
        with self.lock:
            self.pub_torque.publish(random.choice(
                [random.random(), 0, 0, 0, 0]) * max_torque)

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('flaky_door_opener')
    rospy.loginfo('starting flaky_door_opener')
    do = DoorOpener()
    do.spin()
    rospy.loginfo('done')

if __name__=='__main__':
    main()
