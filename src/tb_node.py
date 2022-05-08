#! /usr/bin/env python
# -*- coding: utf-8 -*-
# =============================================================================
# Created By  : Hrushikesh Budhale, Pratik Acharya
# Created Date: Saturday 7 May 2022
# =============================================================================

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import time
import planner
import sys


def pose_callback(msg):
    global rx, ry, rY
    rx = msg.pose.pose.position.x
    ry = msg.pose.pose.position.y
    rq = msg.pose.pose.orientation
    (_, _, rY) = euler_from_quaternion([rq.x, rq.y, rq.z, rq.w])


def main():
    global rx, ry, rY
    rx = ry = rY = 0
    msg=Twist()
    pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
    sub=rospy.Subscriber("/odom", Odometry, pose_callback)
    rospy.init_node('tb_node',anonymous=True)

    # rpm values have been kept lower to eliminate wheel slip
    path = planner.main(sys.argv[1:])
    rospy.loginfo("Started following the path")

    for (x, y) in path:
        x -= 5  # since worlds bottom left is at (-5, -5)
        y -= 5
        while True:
            targetPose = np.array([x-rx, y-ry])
            
            phi = np.arctan2(targetPose[1], targetPose[0]) - rY
            distance = np.linalg.norm(targetPose)
            
            if distance < 0.2: break
            
            # Proportional control
            msg.linear.x = 0.5 * np.clip(distance, -0.5, 0.5)
            msg.angular.z = 1 * phi 
            pub.publish(msg)
            rospy.sleep(0.1)


    # stop the robot and node
    msg=Twist()
    pub.publish(msg)
    rospy.loginfo("Reached the goal location")

    rospy.spin()
    rospy.signal_shutdown("done")
		
if __name__=='__main__':
    main()
	
