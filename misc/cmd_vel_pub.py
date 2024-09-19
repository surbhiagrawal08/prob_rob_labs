
 #!/usr/bin/env python
 # license removed for brevity
import rospy
from geometry_msgs.msg import Twist
    
def talker():
    rospy.init_node('talker')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        t = Twist()
        t.linear.x = 1
        t.linear.y = 0
        t.linear.z = 0
        t.angular.x = 0
        t.angular.y = 0
        t.angular.z = 0.5
        rospy.loginfo(t)
        pub.publish(t)
        rate.sleep()
   
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
