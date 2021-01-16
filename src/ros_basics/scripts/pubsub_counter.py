#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64

counter = 0

def callback_number(msg):
	global counter
	counter += msg.data
	new_msg = Int64()
	new_msg.data = counter
	pub.publish(new_msg)
	rospy.loginfo("I Publish the counter value: %s", counter)

rospy.init_node('number_counter')
pub = rospy.Publisher("/number_count", Int64, queue_size=10)
sub = rospy.Subscriber("/number", Int64, callback_number)
rospy.spin()