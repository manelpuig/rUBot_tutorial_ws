#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
#we have imported the needed libraries

#we define the callback function
def callback_string(msg):
	if msg.data == "Ping":
		#if the published message is Ping, the pong_node will publish Pong on the topic
		new_msg = String()
		new_msg.data = "Pong"
		pub.publish(new_msg)
		rospy.loginfo("I received: " + msg.data + " and I answer: " + new_msg.data) #the call and answer are printed in the terminal
	else:
		#if the published message is NOT Ping, the pong_node will publish Failed! on the topic
		new_msg = String()
		new_msg.data = "Failed!"
		pub.publish(new_msg)
		rospy.loginfo("I received: " + msg. data + " and I answer: " + new_msg.data) #the call and answer are printed in the terminal

def pong():
	#we initialize the node and define the topic from which we are receiving the words (subscribing) and the topic we are publishing into
	rospy.init_node('pong_node', anonymous = True)
	pub = rospy.Publisher("pong_topic", String, queue_size=10)
	sub = rospy.Subscriber("ping_topic", String, callback_string)
	rospy.spin()

if __name__ == '__main__':
    pong()