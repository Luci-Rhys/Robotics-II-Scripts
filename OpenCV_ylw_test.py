#! /usr/bin/env python3

import rospy
import numpy as np
import cv2
from queue import Queue
from math import pi
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

cv_bridge = CvBridge()
cam_queue = Queue(1)
yaw = 0.0
front = left = rear = right = None


'''
Callback
'''
#usbcam callback
def cam_cb(data):

    cnts = None

    if cam_queue.full():
        rospy.loginfo("The queue is full, waitiing to dequeue")
        return

    rospy.loginfo("New photo received")
    cv_image = cv_bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

    if data is None:
        rospy.loginfo("No data ready")
        return
    
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) #Converts color to HSV

    #Thresholds for yellow
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([50, 255, 255])

    yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
    yellow_mask = cv2.erode(yellow_mask, None, iterations=2)
    yellow_mask = cv2.dilate(yellow_mask, None, iterations=2)

    contours = cv2.findContours(yellow_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = contours[0]
    rospy.loginfo(f"Cnts = {cnts}")

    #Prints to console that ball found when yellow object detected
    if cnts:
        rospy.loginfo("Ball found")
    else:
        rospy.loginfo("No ball detected")

    cam_queue.put(cnts)

    rospy.loginfo("New addition to queue")
    

'''
Telemetry code
''' 

#Publisher for the cmd_vel topic
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

#Function moves robot forward until yellow object is detected
def move_forward():
    rate = rospy.Rate(10) #Publish rate
    var = Twist()

    var.linear.x = 0.3
    var.angular.z = 0.0

    rospy.loginfo_once("Before loop")

    #Publishes updated velocities to cmd_vel as long as no yellow item is detected
    while not rospy.is_shutdown(): 
        data = cam_queue.get()
        rospy.loginfo(f"In main, current queue: {data}")
        if len(data) == 0:
            rospy.loginfo("During loop")
            vel_pub.publish(var)
            rate.sleep()

    var.linear.x = 0.0
    vel_pub.publish(var)

'''
Main
'''
def main():
    rospy.init_node('yellow_ball_test')

    #Turtlebot camera subscriber
    cam_sub = rospy.Subscriber("/usb_cam/image_raw", Image, cam_cb)

    # Subscribes to the laser LIDAR to measure distance between robots and other objects
    nav_sub = rospy.Subscriber('/scan', LaserScan, callback, queue_size=1)

    #Subscribes to odometry for rotation
    odom_sub = rospy.Subscriber ('/odom', Odometry, get_rotation)

    move_forward()

    rospy.spin()

if __name__ == '__main__':
     main()
