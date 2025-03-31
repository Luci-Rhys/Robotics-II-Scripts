#! /usr/bin/env python3

# Calen Jones
# Dr. Nicholson
# CSCI 4561


'''
This ROS script is testing the lidar on a Turtlebot3 robot using a queue for threading and separation of services
DISCLAIMER: Debug statements commented out to prevent slowing down program, while still keeping text in case debug statements are needed later
'''

from math import inf
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from queue import Queue

lidar_queue = Queue(1)

# Publishes to cmd vel to control movement
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

#Callback for lidar; retrieves front distance and adds it to queue, returning if the queue is still full by the time it cycles
def callback(msg):
    rospy.loginfo("Enter callback loop")

    if lidar_queue.full():
        # rospy.loginfo("Lidar Queue is full, waiting to dequeue")
        return
         
    # rospy.loginfo("New LIDAR message received") #Debug to ensure messages are still being received
    
    robot_front = msg.ranges[0]
    rospy.loginfo(f"Current front: {robot_front}")
    lidar_queue.put(robot_front)

#Function retrieves front distance from lidar queue and publishes to cmd_vel while that distance is less than 0.3 meters
def go_forward():
    rate = rospy.Rate(2)
    var = Twist() #Published to /cmd_vel publisher to move robot

    var.linear.x = 0.3
    var.angular.z = 0.0

    # rospy.loginfo_once("Before go forward") #Debug statement

    while not rospy.is_shutdown() and lidar_queue.get() > 0.3:
        # rospy.loginfo("Entered go forward loop")
        vel_pub.publish(var)
        rate.sleep()

    var.linear.x = 0.0
    vel_pub.publish(var)


def main():
    rospy.init_node('lidar_test_node') #Node for script

    # Subscribes to the laser LIDAR to measure distance between robots and other objects
    nav_sub = rospy.Subscriber('/scan', LaserScan, callback, queue_size=1)

    go_forward()
    rospy.spin()

if __name__ == "__main__":
    main()
