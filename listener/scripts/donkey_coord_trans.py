#!/usr/bin/env python

import rospy

import tf

from std_msgs.msg import String

from geometry_msgs.msg import PoseStamped, Twist

import copy
 

class test:

   

    def __init__(self):

       

        self.br = tf.TransformBroadcaster()

        self.pub = rospy.Publisher('/donkey/cmd_vel',Twist)

        self.sub = rospy.Subscriber("/mocap_node/Donkey_car/pose",PoseStamped, self.callback)

        self.rate = rospy.Rate(50)

        self.counter = 0

        self.pose_to_send = PoseStamped()

        self.temp = 0

        self.lastPos = PoseStamped()

        self.vel = Twist()

        # self.vel.pose.position.z = 0

 

    def callback(self,data):

        self.counter %= 5

        if self.counter ==0:

            # rospy.loginfo(rospy.get_caller_id() + "%s", data)

            #transform data into

            # x -> x

            # y -> -z

            # z -> y

            #

            self.temp = data.pose.position.y

            self.pose_to_send.header = data.header

            self.pose_to_send.pose.position.x = data.pose.position.x

            self.pose_to_send.pose.position.y = data.pose.position.z

            self.pose_to_send.pose.position.z = -self.temp

            self.temp = data.pose.orientation.y

            self.temp2 = data.pose.orientation.z

            self.pose_to_send.pose.orientation.y = self.temp2

            self.pose_to_send.pose.orientation.x = data.pose.orientation.x

            self.pose_to_send.pose.orientation.z = -self.temp

            self.pose_to_send.pose.orientation.w = data.pose.orientation.w

 
            rospy.loginfo(rospy.get_caller_id() + "%s", self.pose_to_send.header.stamp)
            rospy.loginfo(rospy.get_caller_id() + "%s", self.lastPos.header.stamp)
           

 

            self.vel.linear.x = (self.pose_to_send.pose.position.x - self.lastPos.pose.position.x) / (self.pose_to_send.header.stamp - self.lastPos.header.stamp).to_sec()

            self.vel.linear.y = (self.pose_to_send.pose.position.y - self.lastPos.pose.position.y) / (self.pose_to_send.header.stamp - self.lastPos.header.stamp).to_sec()

 

            self.pub.publish(self.vel)

            rospy.loginfo(rospy.get_caller_id() + "%s", self.pose_to_send)

            rospy.loginfo(rospy.get_caller_id() + "%s", self.vel)

            self.lastPos = copy.deepcopy(self.pose_to_send)

            # self.br.sendTransform((self.pose_to_send.pose.position.x,self.pose_to_send.pose.position.y,self.pose_to_send.pose.position.z),

            #   (self.pose_to_send.pose.orientation.x,self.pose_to_send.pose.orientation.y,

            #   self.pose_to_send.pose.orientation.z,self.pose_to_send.pose.orientation.w),

            #   rospy.Time.now(),"base_link","world")

            self.rate.sleep()

        self.counter +=1

       

       

   

#def listener():

#   rospy.init_node('listener')

#   pub = rospy.Publisher('/mavros/vision_position/pose',PoseStamped)

#   sub = rospy.Subscriber("/mocap_node/drone_3/pose",PoseStamped, callback)

   

#   rospy.spin()

   

def main():

    rospy.init_node('Donkey_cruise_control')

    data_1 = test()

 

    try:

        rospy.spin()

    except KeyboardInterrupt:

        print("exit")

   

if __name__ == '__main__':

    main() 