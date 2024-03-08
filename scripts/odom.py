#!/usr/bin/env python3

from email import header
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import tf2_ros
import geometry_msgs

rospy.init_node('odom_pub', anonymous=False)

odom_pub=rospy.Publisher('mobot/odom', Odometry, queue_size=10)
rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

odom =Odometry()
header=Header()

header.frame_id='odom'

model=GetModelStateRequest()
model.model_name='mobot'

def tf_broadcaster(msg):
    br = tf2_ros.TransformBroadcaster()

    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = 'base_link'
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    
    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w
    
    br.sendTransform(t)

while not rospy.is_shutdown():
    result=get_model_srv(model)

    odom.child_frame_id='base_link'

    odom.pose.pose=result.pose
    odom.twist.twist=result.twist

    header.stamp=rospy.Time.now()
    odom.header=header
    tf_broadcaster(odom)
    odom_pub.publish(odom)