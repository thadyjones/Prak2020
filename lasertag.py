#! /usr/bin/env python

import rospy
import actionlib
import rpc_game_client
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from AprilTagDetectionArray.msg import AprilTagDetectionArray
from sensor_msgs.msg import Image, CameraInfo

lastImage = Image()
cameraInfo = CameraInfo()

def analyse(detectionArray):
    if detectionArray.AprilTagDetection[0] != None :
        global cameraInfo

        playerScore = PlayerScore()
        playerScore.camerainfo = cameraInfo

def saveImage(image):
    global lastImage 
    lastImage = image

def saveCameraInfo(thisCameraInfo):
    global cameraInfo
    cameraInfo = thisCameraInfo

def move_between_client():
 	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
 	client.wait_for_server()

    rospy.Subscriber('tag_detections', AprilTagDetectionArray, analyse)
    rospy.Subscriber('image_rect', Image, saveImage) #TODO Compressed Image statt Image
    rospy.Subscriber('camera_info', CameraInfo, saveCameraInfo)

 	point1 = MoveBaseGoal()
 	point1.target_pose.header.frame_id = "map"
 	point1.target_pose.header.stamp = rospy.Time.now()
 	point1.target_pose.pose.position.x = -10.0 #TODO
 	point1.target_pose.pose.position.y = 2.2 #TODO
 	point1.target_pose.pose.orientation.w = 1.0 

 	point2 = MoveBaseGoal()
 	point2.target_pose.header.frame_id = "map"
 	point2.target_pose.header.stamp = rospy.Time.now()
 	point2.target_pose.pose.position.x = -4.5 #TODO
 	point2.target_pose.pose.position.y = 3.1 #TODO
 	point2.target_pose.pose.orientation.w = 1.0 

 	while not rospy.is_shutdown():
 		client.send_goal(point1)
 		client.wait_for_result()
 		client.send_goal(point2)
 		client.wait_for_result()
 		

if __name__ == '__main__':
 	rospy.init_node('lasertag')
 	move_between_client()
