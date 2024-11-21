#!/usr/bin/env python

import rospy
import numpy as np
import sympy as sp
import math
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from opencv_apps.msg import Point2DArrayStamped
from opencv_apps.msg import Point2D
from tf import transformations



class Pixel_Measurement_Predictor(): #colour and predicted_measurement should be its parameters
    def __init__(self):
        self.initialized = False
        self.gt_sub = rospy.Subscriber('/jackal/ground_truth/pose', PoseStamped, self.prediction_callback)
        self.camera_info_sub = rospy.Subscriber('/front/left/camera_info', CameraInfo, self.camera_info_callback)        
        self.fx = self.fy = self.cx = self.cy = None
        rospy.set_param('landmark_color', 'red')
        #rospy.set_param('landmark_coordinate', [8.5, 5.])
        rospy.set_param('landmark_coordinates', {
            'green': [8.5, 5.0],
            'red': [8.5, -5.],
            'magenta': [-11.5, -5.],
            'yellow': [-11.5, 5.0],
            'cyan': [0.,0.]
        })
        self.landmark_color = rospy.get_param('landmark_color')
        self.pixel_predictor = rospy.Publisher('/'+self.landmark_color +'/predicted_corners', Point2DArrayStamped, queue_size=10)
        rospy.loginfo('/'+self.landmark_color +'/predicted_corners')

        x, y, theta, xl, yl, rl, hl, fx, fy, cx, cy, tx, ty, tz = sp.symbols('x y theta xl yl rl hl fx fy cx cy tx ty tz')


        T_mb = sp.Matrix([[sp.cos(theta), -sp.sin(theta), 0, x],
                         [sp.sin(theta), sp.cos(theta), 0, y],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
        
        T_bo = sp.Matrix([[0, 0, 1, tx], [-1, 0, 0, ty], [0, -1, 0, tz], [0, 0, 0, 1]])
        T_mo = sp.simplify(T_mb*T_bo)

        T_om = sp.simplify(T_mo.inv())

        xc = T_om[3,0]
        yc = T_om[3,1]

        psi = sp.atan2(yl-yc, xl-xc)

        p1m = sp.Matrix([[xl-rl*sp.sin(psi)], [yl+rl*sp.cos(psi)], [0], [1]])
        p2m = sp.Matrix([[xl-rl*sp.sin(psi)], [yl+rl*sp.cos(psi)], [hl], [1]])
        p3m = sp.Matrix([[xl+rl*sp.sin(psi)], [yl-rl*sp.cos(psi)], [0], [1]])
        p4m = sp.Matrix([[xl+rl*sp.sin(psi)], [yl-rl*sp.cos(psi)], [hl], [1]])

        p1o = sp.simplify(T_om*p1m)
        p2o = sp.simplify(T_om*p2m)
        p3o = sp.simplify(T_om*p3m)
        p4o = sp.simplify(T_om*p4m)

        p1o = p1o.row_del(3)
        p2o = p2o.row_del(3)
        p3o = p3o.row_del(3)
        p4o = p4o.row_del(3)

        camera_proj_mat = sp.Matrix([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

        aux1 = sp.simplify(camera_proj_mat*p1o)
        aux2 = sp.simplify(camera_proj_mat*p2o)
        aux3 = sp.simplify(camera_proj_mat*p3o)
        aux4 = sp.simplify(camera_proj_mat*p4o)

        rospy.loginfo(aux4[2])

        final_measurement= sp.simplify(sp.Matrix([[aux1[0]/aux1[2]], [aux1[1]/aux1[2]], [aux2[0]/aux2[2]],
                                                        [aux2[1]/aux2[2]], [aux3[0]/aux3[2]], [aux3[1]/aux3[2]],
                                                        [aux4[0]/aux4[2]], [aux4[1]/aux4[2]]]))

        self.predict_measurement = sp.lambdify((x, y, theta, xl, yl, rl, hl, fx, fy, cx, cy, tx, ty, tz), final_measurement) #8 coordinates

        Hx = final_measurement.jacobian(sp.Matrix([x, y, theta]))

        self.measurement_jac = sp.lambdify((x, y, theta, xl, yl, rl, hl, fx, fy, cx, cy, tx, ty, tz), Hx)
        rospy.loginfo('Initialization complete.')
        self.initialized = True

    def camera_info_callback(self, msg):
            # Extract camera parameters from the CameraInfo message
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]
        #rospy.loginfo(f"Camera Info received: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def prediction_callback(self, gt_pose):
        if self.initialized == True:
            # Get the landmark color and coordinates
            # Assuming the landmark has a color attribute
            landmark_position = rospy.get_param('landmark_coordinate')
            #rospy.loginfo(landmark_position)
            rl = 0.1  # Assuming the landmark is a circle with radius `rl`
            hl = 1  # Height of the landmark if applicable
            gt = PoseStamped()

            # Get the current robot position and orientation
            x_robot = gt_pose.pose.position.x
            y_robot = gt_pose.pose.position.y
            quat_robot = np.array([gt_pose.pose.orientation.x, gt_pose.pose.orientation.y, gt_pose.pose.orientation.z, gt_pose.pose.orientation.w])
            
            roll, pitch, yaw = transformations.euler_from_quaternion(quat_robot)
            #rospy.loginfo(x_robot)
            tx = 0.229
            tz = 0.216

            # Call the measurement model to predict the pixel coordinates
            
            predicted_pixels = self.predict_measurement(
                    x_robot, y_robot, yaw, landmark_position[0], landmark_position[1], rl, hl, 
                    self.fx, self.fy, self.cx, self.cy, tx, 0, tz)

                # Check visibility (if the predicted pixel coordinates are within image bounds)
            rospy.loginfo(predicted_pixels)
            u1, v1, u2, v2, u3, v3, u4, v4 = predicted_pixels
            coord_visibility = [self.is_within_bounds(predicted_pixels[0], predicted_pixels[1]), 
                                self.is_within_bounds(predicted_pixels[2], predicted_pixels[3]),
                                self.is_within_bounds(predicted_pixels[4], predicted_pixels[5]),
                                self.is_within_bounds(predicted_pixels[6], predicted_pixels[7])]
            if all(coord_visibility)==False:
                # Publish the landmark's predicted feature if visible

                rospy.loginfo(f"Landmark {self.landmark_color} out of view.")
            if all(coord_visibility)==True:
                
                pixel_prediction = Point2DArrayStamped()
                pixel_prediction.header.stamp = rospy.get_rostime()
#publish here
                point1 =Point2D()
                point1.x = predicted_pixels[0]
                point1.y = predicted_pixels[1]

                point2 =Point2D()
                point2.x = predicted_pixels[2]
                point2.y = predicted_pixels[3]

                point3 =Point2D()
                point3.x = predicted_pixels[4]
                point3.y = predicted_pixels[5]

                point4 =Point2D()
                point4.x = predicted_pixels[6]
                point4.y = predicted_pixels[7]

                point = [point1, point2, point3, point4]

                pixel_prediction.points = point

                self.pixel_predictor.publish(pixel_prediction)            
       

    def is_within_bounds(self, u, v):
        """Check if the pixel coordinates (u, v) are within the camera's image bounds."""
        image_width = 1024  # pixel width of the camera
        image_height = 768  # pixel height of the camera
        return 0 <= u <= image_width and 0 <= v <= image_height




def main():
    rospy.init_node('pixel_measurement_predictor')
    rospy.loginfo('starting pixel_measurement_predictor')
    Pixel_Measurement_Predictor()
    rospy.spin()
    rospy.loginfo('done')

if __name__=='__main__':
    main()
