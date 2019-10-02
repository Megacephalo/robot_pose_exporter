#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import csv
import datetime
from tf import transformations

class writeToCSV:
    def __init__(self, output_fileName):
        iso_time = datetime.datetime.now().replace(microsecond=0).isoformat()
        self.output_fileName = str( output_fileName ) + '_' + str( iso_time )
        if not self.output_fileName.lower().endswith('.csv'):
            self.output_fileName += '.csv'

        # Insert the metadata
        with open( self.output_fileName, mode = 'w' ) as outFile:
            poseWriter = csv.writer(outFile, delimiter = ',')
            poseWriter.writerow( ['timestamp', 'x(m)', 'y(m)', 'z(m)', 'orientation (rad)'] )

        if rospy.is_shutdown():
            print( '{} created'.format(self.output_fileName) )
        else:
            rospy.loginfo( '{} created'.format(self.output_fileName) )

    def outputPoseToCSV(self, poseStamped):
        if not self.output_fileName:
            err_msg = 'There is no specified outputfile thus nowhere to write. Quitting.'
            if rospy.is_shutdown():
                print(err_msg)
            else:
                rospy.logerr(err_msg)
            return

        with open( self.output_fileName, mode = 'a' ) as outFile:
            pose_stamped = poseStamped
            timestamp = pose_stamped.header.stamp
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            z = pose_stamped.pose.position.z

            quaternion = (
                pose_stamped.pose.orientation.x,
                pose_stamped.pose.orientation.y,
                pose_stamped.pose.orientation.z,
                pose_stamped.pose.orientation.w
            )
            euler = transformations.euler_from_quaternion( quaternion )
            roll, pitch, yaw = euler[0], euler[1], euler[2]

            poseWriter = csv.writer(outFile, delimiter = ',')
            poseWriter.writerow( [timestamp, x, y, z, yaw] )