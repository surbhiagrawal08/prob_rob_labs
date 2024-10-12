#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

# Global variables for state and time
state = 0
now = None
pub_door = None
door_open = None

def feature_mean_callback(msg):
    global door_open
    door_open = msg.data

def log_state():
    rospy.loginfo(f"Current state: {state}")

def state_machine(event):
    global state, now, pub_door, pub2

    log_state()

    # State 0: Initialize and set the time
    if state == 0:
        now = rospy.get_rostime()  # Record the current time
        state = 1  # Transition to state 1
        return

    # State 1: Wait for 0.5 seconds before moving to next state
    if state == 1:
        delta_t = rospy.get_rostime() - now  # Calculate elapsed time
        if delta_t.to_sec() > 0.5:  # Wait for 0.5 seconds
            state = 2  # Transition to state 2
        return

    # State 2: Open door by publishing torque command
    if state == 2:
        rospy.loginfo("Opening the door")
        pub_door.publish(Float64(10))  # Apply torque to open door
        now = rospy.get_rostime()  # Reset the timer
        state = 3  # Transition to state 3
        return

    # State 3: Wait for 10 seconds with the door open
    if state == 3:
        #delta_t = rospy.get_rostime() - now  # Calculate elapsed time
        #if delta_t.to_sec() > 10.0:  # Wait for 10 seconds
        if door_open<=450:
            state = 4  # Transition to state 4
            now = rospy.get_rostime()
        return


    if state == 4:
        cmd_vel = Twist()
        rospy.loginfo("Moving the vehicle")
        delta_t = rospy.get_rostime() - now
        cmd_vel.linear.x = 0.5
        cmd_vel.angular.z = 0
        pub2.publish(cmd_vel)
        if delta_t.to_sec() > 10.0:
            state = 5
        return
    # State 5: Close the door by applying reverse torque
    if state == 5:
        rospy.loginfo("Closing the door")
        pub_door.publish(Float64(-5))  # Apply reverse torque to close door
        state = 6  # Transition to stopping state
        return

    # State 6: End the operation
    if state == 6:
        rospy.loginfo("Operation complete, stopping")
        rospy.signal_shutdown("State machine finished")  # Stop the node

def main():
    global pub_door, pub2

    rospy.init_node('open_door_and_move')
    
    # Publishers
    pub_door = rospy.Publisher('/hinged_glass_door/torque', Float64, queue_size=10)
    pub2  = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/feature_mean', Float64, feature_mean_callback)
    
    rospy.loginfo('Starting state machine')

    # Timer setup (calls state_machine every 0.1 seconds)
    rospy.Timer(rospy.Duration(0.1), state_machine)

    # Keep the program alive until the state machine finishes
    rospy.spin()

if __name__ == '__main__':
    main()
