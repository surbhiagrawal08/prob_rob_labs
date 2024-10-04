#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

#def torque_callback(msg):
    #current_torque_value = msg.data
    #rospy.loginfo(f"torque: {current_torque_value}")

def open_door_and_move():
    pub1 = rospy.Publisher('/hinged_glass_door/torque', Float64, queue_size=10)
    
    #what does queue do here?
    #rospy.loginfo('pub1: {pub1.data}')
    pub2  = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    torque_command = Float64()  #% rospy.get_time()
    torque_command_param =float(rospy.get_param('~/open_door_and_move/torque_command_param', 5.0))
    #torque_command.data = 5
    torque_command.data = torque_command_param #*(rospy.get_time()%10)
    rospy.loginfo('I am about to publish {}'.format(torque_command))
    rospy.sleep(.1)
    for i in range(3):
        pub1.publish(torque_command)
        rospy.sleep(0.1)
    #rospy.Subscriber('/hinged_glass_door/torque', Float64, torque_callback)
    rospy.sleep(10)
    rospy.loginfo('preparing to move')
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.5
    cmd_vel.angular.z = 0
    for i in range(100):
        pub2.publish(cmd_vel)
        rate.sleep()

    cmd_vel.linear.x = 0.0
    pub2.publish(cmd_vel)
    rospy.loginfo('moved through and now stopping')
    rospy.sleep(2)
    rospy.loginfo('closing the door')
    torque_command.data = -1.5
    pub1.publish(torque_command)
    rate.sleep()

        


def main():
    #global pub_door
    rospy.init_node('open_door_and_move') #rospy.init_node('open_door_and_move')
    rospy.loginfo('starting open_door_and_move')
    open_door_and_move()
    #pub_door = rospy.Publisher('/hinged_glass_door/torque', Float64, queue_size=10)
    #pub_door = rospy.Publisher('/hinged_glass_door/torque', Float64, queue_size=10)
    #timer=rospy.Timer(rospy.Duration(0.1), heartbeat)
    rospy.spin()
    rospy.loginfo('done')

if __name__=='__main__':
    main()
