#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

from write_to_csv_lib.write_to_csv import *

node_name = 'save_odom_to_csv'

class saveOdomToCSV:
    def __init__(self):
        self.odomTopic = rospy.get_param('~odom_topic', '/ground_truth/state')
        self.output_csvFileName = rospy.get_param('~output_csv_fileName', '/home/charly_huang/data_log/ground_truth')

        self.realPose = Pose()
        self.poseToCSVSaver = writeToCSV( self.output_csvFileName )

        rospy.Subscriber(self.odomTopic, Odometry, self.odomCallback)
        rospy.spin()

    def odomCallback(self, odom_msg):
        self.realPose = odom_msg.pose.pose

        if self.realPose:
            self.poseToCSVSaver.savePoseToCSV( self.realPose )
        

if __name__=='__main__':

    rospy.init_node( node_name )
    rospy.loginfo('{} is launched'.format( node_name ) )

    poseSaver = saveOdomToCSV()

    rospy.loginfo('Done!')