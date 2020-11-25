#!/usr/bin/env python
#coding=utf-8
import rospy
from  nav_msgs.msg  import Odometry
from geometry_msgs.msg import PoseStamped,Point

rospy.init_node('ts')
# var=rospy.wait_for_message('/odom',Odometry,timeout=5)
# print(var.pose.pose.position.x, var.pose.pose.position.y)
goalMsg = PoseStamped()
goalPub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
goalMsg.header.frame_id = '/map'
goalMsg.header.seq = 0
# Destination
goalMsg.pose.position.x =1.0
goalMsg.pose.position.y = 1.7
goalMsg.pose.orientation.z = 0.0
goalMsg.pose.orientation.w = 0.0
rate = rospy.Rate(1)
rospy.loginfo('x=%f , y=%f',goalMsg.pose.position.x,goalMsg.pose.position.y)

while not rospy.is_shutdown():
    goalMsg.header.stamp = rospy.Time.now()
    goalPub.publish(goalMsg)
    rate.sleep()

