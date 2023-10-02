#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import random as rnd
#we have imported the needed libraries

words = ["Ping", "PingPong", "Robotics", "Biomedical Engineering", "Surgery", "Robot"] #list of words from which a random word will be sennt
def ping():
	global words
	#we initialize the node and define the ROS topic
	rospy.init_node("ping_node", anonymous=True)
	pub = rospy.Publisher("ping_topic", String, queue_size=10)
	rate = rospy.Rate(1) #we define the rate as 1 publishing every second

	while not rospy.is_shutdown():
		msg = String()
		msg.data = rnd.choice(words) #we define the message as one of the words from the list chosen randomly
		pub.publish(msg) #we publish it to the topic
		rate.sleep() #a word will be sent every second

if __name__ == '__main__':
    try:
        ping()
    except rospy.ROSInterruptException:
        pass