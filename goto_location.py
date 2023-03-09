#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, PoseWithCovarianceStamped, Quaternion, PoseStamped
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import *
import std_srvs.srv
from std_msgs.msg import String, Float32, Empty
import time


def goto_location(location,sensitivity):
    goalReached = False

    while(not goalReached):

        goalReached = moveToGoal(location,sensitivity)
        if (goalReached):
            rospy.loginfo("Reached destination!")


def moveToGoal(location,sensitivity):
    global move
    move = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    while(not move.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up")
    simplegoal = MoveBaseGoal()
    simplegoal.target_pose.header.frame_id = "map"
    simplegoal.target_pose.header.stamp = rospy.Time.now()
    goal_location_coordinates = location.split(",")
    simplegoal.target_pose.pose.position =  Point(float(goal_location_coordinates[0]),float(goal_location_coordinates[1]),0)
    simplegoal.target_pose.pose.orientation.x = 0.0
    simplegoal.target_pose.pose.orientation.y = 0.0
    # if(sensitivity != 0):
    simplegoal.target_pose.pose.orientation.z = float(goal_location_coordinates[2])
    simplegoal.target_pose.pose.orientation.w = float(goal_location_coordinates[3])
    # else:
    #     simplegoal.target_pose.pose.orientation.z = 0.0
    #     simplegoal.target_pose.pose.orientation.w = 0.0
    rospy.loginfo("Sending Next goal location ")
    move.send_goal(simplegoal)
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    msg = Twist()
    speed = 0.1
    msg.linear.x = speed
    pub.publish(msg)

    move.wait_for_result(rospy.Duration(60))
    if(move.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("This is the end of navigation")
        return True

    else:
        rospy.loginfo("The robot failed to reach the destination")
        return False




if __name__ == '__main__':
    try:
        rospy.init_node('asp_navigator_py')
        time.sleep(5)
        while True:
            # goto_location("-0.02, -6.7,0.01,0.0",0) # location N00
            # goto_location("0.71,-0.26,0.0034,0.0",0) # Location 3 Lab
            goto_location("-8.5,20.4,0.0034,0.0",0) # Outside lift
            goto_location("-17.7,18.4,0.0034,0.0",0) # Location 3 Lab
            # goto_location("8.34,22.2,0.0034,0.0",0) # Behind warehouse area back
            # goto_location("-33.6,14,0.0034,0.0",0) # Front Stairs center
            # goto_location("-42.9,13.4,0.0034,0.0",0) # Back Stairs center

        print "end"

    except rospy.ROSInterruptException:
            print "finished!"
