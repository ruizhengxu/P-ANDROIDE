"""
Exemple of a complete msg:
---
header: 
	seq: 36817
	stamp: 
		secs: 1343
		nsecs: 971000000
	frame_id: "odom"
child_frame_id: "base_footprint"
pose: 
	pose: 
		position: 
			x: 0.237045993794
			y: -1.77755008158
			z: -0.00100242735835
		orientation: 
			x: 0.00180726869465
			y: -0.00341181097251
			z: -0.466434446682
			w: -0.88454734202
	covariance: [1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001]
twist: 
	twist: 
		linear: 
			x: -1.60580832086e-06
			y: 1.32810955322e-05
			z: 0.0
		angular: 
			x: 0.0
			y: 0.0
			z: 0.000203054588972
	covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
"""

import os
import rospy
from nav_msgs.msg import Odometry
import json

class Pos():
	def __init__(self):
		self.data__ = ""
		rospy.init_node('get_pos_and_twist')
		odom_sub = rospy.Subscriber('/odom', Odometry, self.callback)

		rospy.spin()

	def callback(self, msg):
		#print(msg)
		self.data__ = {
      'pose' : {
        'position' : {
          'x' : msg.pose.pose.position.x,
          'y' : msg.pose.pose.position.y,
          'z' : msg.pose.pose.position.z
        },
        'orientation' : {
          'x' : msg.pose.pose.orientation.x,
          'y' : msg.pose.pose.orientation.y,
          'z' : msg.pose.pose.orientation.z,
          'w' : msg.pose.pose.orientation.w
        }
      },
      'twist' : {
        'linear' : {
          'x' : msg.twist.twist.linear.x,
          'y' : msg.twist.twist.linear.y,
          'z' : msg.twist.twist.linear.z
        },
        'angular' : {
          'x' : msg.twist.twist.angular.x,
          'y' : msg.twist.twist.angular.y,
          'z' : msg.twist.twist.angular.z
        }
      }
    }

		
		with open('data.json', 'w') as fp:
			json.dump(self.data__, fp)

	def getData(self):
		return self.data__

p = Pos()