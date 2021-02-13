#!/usr/bin/env python
# -*- coding: utf-8 -*-

# echo.py: sample script to print ros message to terminal
# Author: Ravi Joshi
# Date: 2020/03/02

# import modules
import rospy
from ros_openpose.msg import Frame

POSE_BODY_25_BODY_PARTS = ["Nose", "Neck", "RShoulder", "RElbow", "RWrist", "LShoulder","LElbow", "LWrist", "MidHip", "RHip", "RKnee", "RAnkle", "LKnee", "LAnkle", "REye", "LEye", "REar", "LEar", "LBigToe", "LSmallToe", "LHeel", "RBigToe", "RSmallToe", "RHeel", "Background"]

def callback(msg):
    #text = [bodyPart.pixel for person in msg.persons for bodyPart in person.bodyParts]
    for per_ind, person in enumerate(msg.persons):
      for bp_ind, bodyPart in enumerate(person.bodyParts):
        if bp_ind == 3:
          text = "[Per:" + str(per_ind) + "] " + POSE_BODY_25_BODY_PARTS[bp_ind] + " (" + str(bp_ind) + "): " + str(bodyPart.point.x) + " " + str(bodyPart.point.y) + " " + str(bodyPart.point.z)
          rospy.loginfo('%s' % text)

def main():
    rospy.init_node('echo', anonymous=False)

    # read the parameter from ROS parameter server
    frame_topic = rospy.get_param('~pub_topic')

    rospy.Subscriber(frame_topic, Frame, callback)

    rospy.spin()


if __name__ == '__main__':
    main()
