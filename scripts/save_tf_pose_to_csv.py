#! /usr/bin/env python

import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped
import time

from write_to_csv_lib.write_to_csv import *

class tf_extract_pose:
    def __init__(self):
        self.listener = tf.TransformListener()

        self.from_frame = rospy.get_param('from_frame', 'map')
        self.to_frame = rospy.get_param('to_frame', 'odom')
        self.wait_thresh = rospy.get_param('tf_timeout', 10.0)
        self.pubPose_topic = rospy.get_param('pub_pose_topic', 'estimated_state')
        self.outputCSVFile = rospy.get_param('output_csv_file', '/home/charly_huang/data_log/robot_pose')

        self.pose_pub = rospy.Publisher(self.pubPose_topic, PoseStamped, queue_size = 10)

        # Write poses to CSV file
        self.poseWriter = writeToCSV( self.outputCSVFile )

        rospy.loginfo('Waiting to acquire TF transform')
        self.listener.waitForTransform(self.from_frame, self.to_frame, rospy.Time(0), rospy.Duration(self.wait_thresh))
        rospy.loginfo('TF transform acquired. Proceed.')

        self.robot_pose = PoseStamped()
        self.robot_pose.header.frame_id = self.from_frame

        self.acquirePose()


    def acquirePose(self):
        trans = None
        rot = None
        while not rospy.is_shutdown():
            # now = self.listener.getLatestCommonTime(self.from_frame, self.to_frame)
            now = rospy.Time.now()

            try:
                trans, rot = self.listener.lookupTransform(self.from_frame, self.to_frame, now)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            self.robot_pose.header.stamp = now
            self.robot_pose.pose.position.x = trans[0]
            self.robot_pose.pose.position.y = trans[1]
            self.robot_pose.pose.position.z = trans[2]

            self.robot_pose.pose.orientation.x = rot[0]
            self.robot_pose.pose.orientation.y = rot[1]
            self.robot_pose.pose.orientation.z = rot[2]
            self.robot_pose.pose.orientation.w = rot[3]

            self.pose_pub.publish( self.robot_pose )

            # Output to CSV file
            self.poseWriter.outputPoseToCSV( self.robot_pose )
            


if __name__=='__main__':
    node_name = 'robot_pose_exporter'
    rospy.init_node( node_name )
    rospy.loginfo('{} is launched'.format(node_name))

    tf_extract_pose()

    rospy.loginfo('Done!')