#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


class test:
	
	def __init__(self):
		self.pub = rospy.Publisher('/mavros/vision_pose/pose',PoseStamped,queue_size = 1)
		#self.pub = rospy.Publisher('/mavros/vision_pose/pose',PoseStamped)
		self.sub = rospy.Subscriber("/mocap_node/drone_3/pose",PoseStamped, self.callback)
		self.rate = rospy.Rate(30)
		self.counter = 0
		self.pose_to_send = PoseStamped()
		self.temp = 0
	def callback(self,data):
		self.counter %= 5
		if self.counter ==0:
			#transform data into
			# x -> x
			# y -> -y
			# z -> -z
			self.temp = data.pose.position.y
			self.pose_to_send = data
			self.pose_to_send.pose.position.y = data.pose.position.x
			self.pose_to_send.pose.position.x = -data.pose.position.z
			self.pose_to_send.pose.position.z = -self.temp
			rospy.loginfo(rospy.get_caller_id() + "%s", data)
			self.pub.publish(self.pose_to_send)
			self.rate.sleep()
		self.counter +=1
		
		
	
#def listener():
#	rospy.init_node('listener')
#	pub = rospy.Publisher('/mavros/vision_position/pose',PoseStamped)
#	sub = rospy.Subscriber("/mocap_node/drone_3/pose",PoseStamped, callback)
	
#	rospy.spin()
	
def main():
	rospy.init_node('listener')
	data_1 = test()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("exit")
	
if __name__ == '__main__':
	main()
