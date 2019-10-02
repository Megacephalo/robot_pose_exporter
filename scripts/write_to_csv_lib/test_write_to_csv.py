#! /usr/bin/env python

from write_to_csv import *
import rospy
from geometry_msgs.msg import PoseStamped

if __name__=='__main__':
    rospy.init_node('test_write_to_csv')
    rospy.loginfo('test_write_to_csv is launched')

    writer = writeToCSV('/home/charly_huang/data_log/dummy')

    # writer2 = writeToCSV('dummy.csv')

    # Input a couple of poseStamped to see how it goes.
    pose1 = PoseStamped()
    pose1.header.stamp = rospy.get_rostime()
    pose1.pose.position.x, pose1.pose.position.y, pose1.pose.position.z = 1.0, 2.0, 3.0
    pose1.pose.orientation.x = 0.0 
    pose1.pose.orientation.y = 0.0
    pose1.pose.orientation.z = 0.0
    pose1.pose.orientation.w = 1.0
    
    writer.outputPoseToCSV( pose1 )

    pose2 = PoseStamped()
    pose2.header.stamp = rospy.get_rostime()
    pose2.pose.position.x, pose2.pose.position.y, pose2.pose.position.z = 0.0, 0.0, 0.0
    pose2.pose.orientation.x = 0.0 
    pose2.pose.orientation.y = 0.0
    pose2.pose.orientation.z = 0.7071068
    pose2.pose.orientation.w = 0.7071068

    writer.outputPoseToCSV( pose2 )




