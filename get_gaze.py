#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, PoseWithCovarianceStamped, Quaternion, PoseStamped
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sound_play.libsoundplay import SoundClient
import time
from datetime import datetime

# writes the table stuff to a txt file from unity so we can parse that in our POMDP 

speaker = SoundClient()
last_table = ""
last_calltime = datetime.strptime("00:00:00","%H:%M:%S")


def callback(data):
	tablefile = open("table.txt", "a")
	global last_calltime, last_table
	table = data.data
	t = time.localtime()
	tf = time.strftime("%H:%M:%S",t)
	current_time = datetime.strptime(tf, "%H:%M:%S")
	delta_time = current_time - last_calltime

	if abs(delta_time.total_seconds()) >= 3:
		if "table1" in table:
			if last_table != "table1":
				print("table1")
				last_table = "table1"
				colors.write("\n")
				colors.write("\ntable1")
				colors.close()

		elif "table2" in table:
			if last_table != "table2":
				print("")
				last_table = "table2"
				colors.write("\n")
				colors.write("\ntable2")
				colors.close()
				



		t = time.localtime()
		tf = time.strftime("%H:%M:%S",t)
		last_calltime = datetime.strptime(tf, "%H:%M:%S")


	return last_table


if __name__ == '__main__':

	try:
		rospy.init_node('speak_object')
		rospy.Subscriber ('/gaze_vector',String, callback)
		rospy.spin()

	except rospy.ROSInterruptException:
		print "finished!"

