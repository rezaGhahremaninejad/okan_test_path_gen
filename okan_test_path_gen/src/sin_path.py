#!/usr/bin/env python2

import rospy
from std_msgs.msg import String

import numpy as np
import math as math
from numpy.linalg import inv

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose, Point, Quaternion, TwistWithCovariance, PoseWithCovariance 

import matplotlib.pyplot as plt


def straight_path():
    path = Path()
    goal_odom = Odometry() 
    #Vehicle linear speed in x, y, z in m/s

    rospy.init_node('straight_path', anonymous=True)
    t = rospy.get_time()
    pathPub = rospy.Publisher('/path', Path, queue_size=1)
    odom_pub = rospy.Publisher("/goal_odom", Odometry, queue_size=50)
    rate = rospy.Rate(10) # 10hz
    
    xp_target = rospy.get_param('target_destination', '150')
    maximum_speed = rospy.get_param('maximum_speed', '2')
    sin_gain = rospy.get_param('sin_gain', '20')
    sin_freq = rospy.get_param('sin_freq', '0.05')
    just_sim = rospy.get_param('just_sim', '0')

    while not rospy.is_shutdown():

        xpo = 0
        ypo = 0
        zpo = 0
        wpo = 1

        vx = 0
        vy = 0
        vz = 0
        
        x2p = []
        y2p = []
        v2p = []
        vx2p = []
        vy2p = []
        # vehicle angular velocity in rad/s
        wx = 0
        wy = 0
        wz = 0

        lenght = int(xp_target)
        v_max = float(maximum_speed)
        gain = float(sin_gain)
        freq = float(sin_freq)

        for xp in range(0, lenght):

            yp = gain*math.sin(freq*xp)
            zp = 0
            x2p.append(xp)
            y2p.append(yp)
            if float(xp) == 0:
                estimated_heading = 0
            elif xp == 1:
                estimated_heading = math.atan(float(yp)/float(xp))
            else:
                estimated_heading = math.atan(float(y2p[xp] - y2p[xp - 1])/float(x2p[xp] - x2p[xp - 1]))

            if xp == 0 or xp == lenght:
                vx = 0
                vy = 0
            elif xp == 1 or xp == lenght - 1:
                vx = 0.25*v_max*math.cos(estimated_heading)
                vy = 0.25*v_max*math.sin(estimated_heading)
            elif xp == 2 or xp == lenght - 2:
                vx = 0.5*v_max*math.cos(estimated_heading)
                vy = 0.5*v_max*math.sin(estimated_heading)
            elif xp == 3 or xp == lenght - 3:
                vx = 0.75*v_max*math.cos(estimated_heading)
                vy = 0.75*v_max*math.sin(estimated_heading)
            else: 
                vx = v_max*math.cos(estimated_heading)
                vy = v_max*math.sin(estimated_heading)

            vz = 0

            vx2p.append(vx)
            vy2p.append(vy)

            if estimated_heading == 0:
                v2p.append(vx)                
            else:
                v2p.append(float(vy)/float(math.sin(estimated_heading)))
            #v2p.append(vx)
            #rospy.loginfo('***************** xp/xp_target: [%f]', float(xp)/float(xp_target))

            quaternion = quaternion_from_euler(xpo, ypo, estimated_heading)

            pose = PoseStamped()
            poseWithCov = PoseWithCovariance()
            twistWithCov = TwistWithCovariance()

            pose.header.frame_id = "map"
            pose.pose.position.x = float(xp)
            pose.pose.position.y = float(yp)
            pose.pose.position.z = float(zp)
            pose.pose.orientation.x = float(quaternion[0])
            pose.pose.orientation.y = float(quaternion[1])
            pose.pose.orientation.z = float(quaternion[2])
            pose.pose.orientation.w = float(quaternion[3])

            poseWithCov.pose.position.x = float(xp)
            poseWithCov.pose.position.y = float(yp)
            poseWithCov.pose.position.z = float(zp)
            poseWithCov.pose.orientation.x = float(quaternion[0])
            poseWithCov.pose.orientation.y = float(quaternion[1])
            poseWithCov.pose.orientation.z = float(quaternion[2])
            poseWithCov.pose.orientation.w = float(quaternion[3])
 
            twistWithCov.twist.linear.x = vx
            twistWithCov.twist.linear.y = vy
            twistWithCov.twist.linear.z = vz
            twistWithCov.twist.angular.x = wx
            twistWithCov.twist.angular.y = wy
            twistWithCov.twist.angular.z = wz

            pose.header.seq = path.header.seq + 1
            path.header.frame_id = "map"
            path.header.stamp = rospy.Time.now()
            pose.header.stamp = path.header.stamp

            goal_odom.header.frame_id = "map"
            goal_odom.header.stamp = rospy.Time.now()
            goal_odom.header.stamp = goal_odom.header.stamp

            path.poses.append(pose)
            goal_odom.pose = poseWithCov
            goal_odom.twist = twistWithCov
            odom_pub.publish(goal_odom)
            #rospy.loginfo('***************** sin_freq: [%s]', sin_freq)
        if float(just_sim) == 1:

            pathPub.publish(path)
            plt.plot(x2p, y2p, label='Y(m) vs X(m)')
            plt.legend()
            plt.show()
        
# WE may need to subscribe to localization ersult since we need heading toward X and Y axis to prepare vx and vy for twist data of the nav_msg/Odometry

    # spin() simply keeps python from exiting until this node is stopped
        #rospy.spin()
        rate.sleep()

if __name__ == '__main__':
    straight_path()
