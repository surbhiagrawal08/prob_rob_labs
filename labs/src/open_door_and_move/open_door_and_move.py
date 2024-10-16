#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import numpy as np

# Global variables for state and time
state = 0
now = None
pub_door = None
feature_mean = None
pub_door_torque = None
door_torque = 0
door_torque_flag = False
i = 0
door_torque_to_give = 0
belief = np.array([[0.5], [0.5]]) #open, closed

#Measurement_Matrix = np.array([[1223/1230, 72/1181], [7/1230, 1109/1181]]) #found using measurement node, order=open, closed
Measurement_Matrix = np.array([[744/(549+744), 357/(911+357)], [549/(549+744), 911/(357+911)]])
Prediction_Model = np.array([[1, 0.1], [0, 0.9]])

def feature_mean_callback(msg):
    global feature_mean
    feature_mean = msg.data

def torque_callback(msg):
    global door_torque
    door_torque = msg.data

def log_state():
    rospy.loginfo(f"Current state: {state}")

def bayesian_filter_update(Measurement_Matrix, Prediction_Model, measurement):
    global belief
    belief = np.matmul(Prediction_Model, belief)
    un_normalized_posterior = (Measurement_Matrix * \
    np.repeat(belief, 2, axis=1).transpose())[measurement, :]
    belief = np.array([un_normalized_posterior/sum(un_normalized_posterior)]).transpose()
    return
   
def state_machine(event):
    global state, now, pub_door, pub2, i, pub_door_torque, door_torque, door_torque_to_give, door_torque_flag

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
        #rospy.loginfo("Opening the door")
        #pub_door.publish(Float64(10))  # Apply torque to open door
        #now = rospy.get_rostime()  # Reset the timer
        state = 3  # Transition to state 3
        return

    # State 3: Wait for 10 seconds with the door open
    if state == 3:
        #delta_t = rospy.get_rostime() - now  # Calculate elapsed time
        #if delta_t.to_sec() > 10.0:  # Wait for 10 seconds
        #if door_open<=455:
 
        if door_torque_flag==True:            
            #rospy.loginfo("im here 2")
            pub_door_torque.publish(Float64(door_torque_to_give))

        if door_torque_flag==False:
            #rospy.loginfo("im here")
            pub_door.publish() 
            #rospy.loginfo(f"Current door_torque: {door_torque}")
            if door_torque>1:
                #rospy.loginfo("@@@")
                door_torque_flag=True
                #rospy.loginfo(f"Current door_torque: {door_torque}")
                door_torque_to_give = door_torque
                return
      
        # if door_torque_flag==False:
        #     pub_door.publish()
        # #    rospy.loginfo("im here")
        #     if door_torque>=1:
        #         door_torque_to_give = door_torque
        #         door_torque_flag=True
        #         rospy.loginfo("im here 2")
                
        # else: 
        #     rospy.loginfo(door_torque_to_give)    
        
        if feature_mean<=452:
            measurement = 0
    
        if feature_mean>452:
            measurement = 1
       
        bayesian_filter_update(Measurement_Matrix, Prediction_Model, measurement)
        i = i+1
        rospy.loginfo(belief[0])
        if belief[0]>0.99:
            rospy.loginfo(i)
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
        pub_door_torque.publish(Float64(-5))  # Apply reverse torque to close door
        state = 6  # Transition to stopping state
        return

    # State 6: End the operation
    if state == 6:
        rospy.loginfo("Operation complete, stopping")
        rospy.signal_shutdown("State machine finished")  # Stop the node
        

def main():
    global pub_door, pub2, pub_door_torque, door_torque, door_torque_to_give

    rospy.init_node('open_door_and_move')
    
    # Publishers
    pub_door_torque = rospy.Publisher('/hinged_glass_door/torque', Float64)
    pub_door = rospy.Publisher('/door_open', Empty, queue_size=10)
    pub2  = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Subscribers
    rospy.Subscriber('/feature_mean', Float64, feature_mean_callback)
    rospy.Subscriber('/hinged_glass_door/torque', Float64, torque_callback)
    
    rospy.loginfo('Starting state machine')

    # Timer setup (calls state_machine every 0.1 seconds)
    rospy.Timer(rospy.Duration(0.1), state_machine)

    # Keep the program alive until the state machine finishes
    rospy.spin()

if __name__ == '__main__':
    main()
