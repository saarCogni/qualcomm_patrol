#!/usr/bin/env python  
import rospy
import math
import tf
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped


if __name__ == '__main__':
    rospy.init_node('base_link_publisher_node')

    listener = tf.TransformListener()
    
    pose_pub = rospy.Publisher('/base_link_pengo', PoseStamped, queue_size=10)   

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            print(str(rot))

            nimbus_pose = PoseStamped()
            nimbus_pose.header.stamp = rospy.Time.now()
            nimbus_pose.header.frame_id = "map"
            nimbus_pose.pose.position.x = trans[0]
            nimbus_pose.pose.position.y = trans[1]
            nimbus_pose.pose.position.z = trans[2]
           
            nimbus_pose.pose.orientation.x = rot[0]
            nimbus_pose.pose.orientation.y = rot[1]
            nimbus_pose.pose.orientation.z = rot[2]
            nimbus_pose.pose.orientation.w = rot[3]

            pose_pub.publish(nimbus_pose)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

      

        rate.sleep()
