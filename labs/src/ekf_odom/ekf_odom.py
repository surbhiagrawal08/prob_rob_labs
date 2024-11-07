#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
from tf import transformations
from std_msgs.msg import Float64


class OdometryEKFNode:
    def __init__(self):
        #rospy.init?
        self.odom_pub = rospy.Publisher('/ekf_odom', Odometry, queue_size=10) 
        imu_sub = Subscriber('/imu/data', Imu) #this is what will be used for prediction
        self.state_cov_pub1 = rospy.Publisher('/x_var', Float64, queue_size=10)
        self.state_cov_pub2 = rospy.Publisher('/y_var', Float64, queue_size=10)
        self.state_cov_pub3 = rospy.Publisher('/theta_var', Float64, queue_size=10)
        self.state_cov_pub4 = rospy.Publisher('/v_var', Float64, queue_size=10)
        self.state_cov_pub5 = rospy.Publisher('/w_var', Float64, queue_size=10)

        joint_sub = Subscriber('/joint_states', JointState)
        cmd_vel_sub = Subscriber('/cmd_vel', Twist)

        self.ats = ApproximateTimeSynchronizer([imu_sub, joint_sub, cmd_vel_sub], queue_size=10, slop=0.1, allow_headerless=True)
        self.ats.registerCallback(self.callback)
        self.state = np.zeros(5).T #state is in order [x, y, theta, V, omega]
        self.state_cov = 0.1*np.eye(5) #initial state known well
        self.prev_time = None

    def callback(self, imu_data, joint_state, cmd_vel):
        current_t = imu_data.header.stamp
        current_t_sec = current_t.to_sec()
        if self.prev_time is None:
            self.prev_time = current_t_sec
            return

        dt = current_t_sec - self.prev_time
        self.prev_time = current_t_sec
        measurement = np.zeros(5).T
        z_cov = 10*np.eye(5)

        measurement[:4] = joint_state.velocity
        measurement[-1] = imu_data.angular_velocity.z
        z_cov[-1,-1] = imu_data.angular_velocity_covariance[8]      
        
        predicted_state, predicted_cov =  self.predict(cmd_vel, dt)
        self.innovation(predicted_state, predicted_cov, measurement, z_cov)
        self.publish_odometry(current_t)

    def predict(self, cmd_vel, dt):
        A = np.zeros(5).T
        #use input knowledge and motion model to predict
        A[0] = self.state[0] + self.state[3]*np.cos(self.state[2])*dt
        A[1] = self.state[1] + self.state[3]*np.sin(self.state[2])*dt
        A[2] = self.state[2] + self.state[4]*dt
        A[3] = 0.986*self.state[3]
        A[4] = 0.95*self.state[4] 

        B = np.array([[0, 0, 0, 0.014, 0], [0, 0, 0, 0, 0.067]]).T
        u = np.array([cmd_vel.linear.x, cmd_vel.angular.z]).T
        state_jacobian = np.array([[1, 0, -self.state[3]*dt*np.sin(self.state[2]), dt*np.cos(self.state[2]), 0],
                                  [0, 1, self.state[3]*dt*np.cos(self.state[2]), dt*np.sin(self.state[2]), 0],
                                  [0, 0, 1, 0, dt], [0, 0, 0, 0.986, 0], [0, 0, 0, 0, 0.95]])
                                
        pred_state = A + B@u
        input_cov = 0.01*np.eye(2)
        pred_state_cov = state_jacobian@self.state_cov@(state_jacobian.T) + B@input_cov@(B.T)

        return pred_state, pred_state_cov
    
    def innovation(self, pred_state, pred_state_cov, measurement, z_cov):
        C = np.zeros((5,5))
        rw = 0.1
        R = 0.323/2
        C[:4, 3] = 1/rw
        C[:, 4] = R/rw
        C[-1, -1] = 1
        C[4,1] = -1*C[4,1]
        C[4,3] = -1*C[4,3]
        Kalman_gain = pred_state_cov@(C.T)@(np.linalg.inv(C@pred_state_cov@(C.T)+z_cov))
        self.state = pred_state + Kalman_gain@(measurement - C@pred_state)
        self.state_cov = (np.eye(5)- Kalman_gain@C)@pred_state_cov

    def publish_odometry(self, current_t):
        message = Odometry()
        orientation_matrix = np.array([[np.cos(self.state[2]), -np.sin(self.state[2]), 0, 0],
                                      [np.sin(self.state[2]), np.cos(self.state[2]), 0, 0],
                                      [0, 0, 1, 0], [ 0, 0, 0, 1]])

        message.header.stamp = current_t
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"

        message.pose.pose.position = Point(self.state[0], self.state[1], 0)        
        message.pose.pose.orientation = Quaternion(*transformations.quaternion_from_matrix(orientation_matrix))
        
        #defining pose covarance matrix
        pose_cov = np.zeros((6,6))
        pose_cov[0, 0] = self.state_cov[0,0]
        pose_cov[0,1], pose_cov[1, 0] = self.state_cov[0,1], self.state_cov[0,1]
        pose_cov[1,1] = self.state_cov[1, 1]
        pose_cov[0, -1], pose_cov[-1, 0] = self.state_cov[0, 2], self.state_cov[0, 2]
        pose_cov[1, -1], pose_cov[-1 ,1] = self.state_cov[1, 2], self.state_cov[1, 2]
        pose_cov[-1, -1] = self.state_cov[2, 2]

        message.pose.covariance = pose_cov.flatten().tolist()

        message.twist.twist.linear.x = self.state[3]
        message.twist.twist.angular.z = self.state[4]

        #defining twist covariance matrix
        twist_cov = np.zeros((6,6))
        twist_cov[0, 0] = self.state_cov[3, 3]
        twist_cov[0, -1], twist_cov[-1, 0] = self.state_cov[-1, 3], self.state_cov[-1, 3]
        twist_cov[-1, -1] = self.state_cov[-1, -1]

        message.twist.covariance = twist_cov.flatten().tolist()
        msg1 = Float64()
        msg1.data = self.state_cov[0,0]
        msg2 = Float64()
        msg2.data = self.state_cov[1,1]
        msg3 = Float64()
        msg3.data = self.state_cov[2,2]
        msg4 = Float64()
        msg4.data = self.state_cov[3,3]
        msg5 = Float64()
        msg5.data = self.state_cov[4,4]
        self.state_cov_pub1.publish(msg1)
        self.state_cov_pub2.publish(msg2)
        self.state_cov_pub3.publish(msg3)
        self.state_cov_pub4.publish(msg4)
        self.state_cov_pub5.publish(msg5)
        self.odom_pub.publish(message)



def main():
    rospy.init_node('ekf_odom')
    odom_node = OdometryEKFNode()
    rospy.loginfo('starting ekf_odom')
    rospy.spin()
    rospy.loginfo('done')

if __name__=='__main__':
    main()
