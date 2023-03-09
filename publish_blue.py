#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
import math
from nav_msgs.msg import Path

current_robot_pose_x = 0
current_robot_pose_y = 0
current_robot_pose_z = 0

path = []


should_publish_pose = False
should_publish_path = False


# def make_ar_plan(robot_goal_location):
#     global start_x
#     global start_y
#     robot_goal_location_coordinates = robot_goal_location.split(",")
#     try:
#         rospy.wait_for_service('/move_base/make_plan')
#         make_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
#         with open('robot_location.txt','a+') as robot_loc:
#             robot_loc.seek(0)
#             current_robot_location = robot_loc.readline()
#             robot_loc.close()
#             current_robot_location_coordinates = current_robot_location.split(",")
#             start_x = current_robot_location_coordinates[0]
#             start_y = current_robot_location_coordinates[1]

#         start = PoseStamped()
#         goal = PoseStamped()
#         start.header.frame_id = "map"
#         goal.header.frame_id = "map"
#         tolerance = 0.0
#         start.pose.position.x= float(start_x)
#         start.pose.position.y= float(start_y)
#         goal.pose.position.x = float(robot_goal_location_coordinates[0])
#         goal.pose.position.y = float(robot_goal_location_coordinates[1])
#         plan_response = make_plan(start = start, goal = goal, tolerance = tolerance)
#         poses = plan_response.plan.poses


def callback(data):
    global should_publish_pose
    global current_robot_pose_x, current_robot_pose_y, current_robot_pose_z


    new_x = data.pose.pose.position.x
    new_y = data.pose.pose.position.y
    new_z = data.pose.pose.position.z

    dist = math.sqrt( (new_x - current_robot_pose_x)**2 + (new_y - current_robot_pose_y)**2 + (new_z - current_robot_pose_z)**2)

    if(dist > 0.1):
        current_robot_pose_x = data.pose.pose.position.x
        current_robot_pose_y = data.pose.pose.position.y
        current_robot_pose_z = data.pose.pose.position.z
        should_publish_pose = True
    else:
        should_publish_pose = True


def pathcallback(data):
    global path
    # print (len(data.poses))

    pose_x = 0
    pose_y = 0
    pose_z = 0
    path = []
    for pose in data.poses:
        # print pose
        pose_x = pose.pose.position.x
        pose_y = pose.pose.position.y
        # pose_z = pose.pose.position.z

        current_pose_point = str(pose_x)+"|"+str(pose_y)
        path.append(current_pose_point)


def talker():
    global should_publish_pose
    rospy.init_node('test_subscribe', anonymous=True)

    rate = rospy.Rate(10) # 1hz
    pub = rospy.Publisher('pose_string_blue_robot', String, queue_size=10)
    pub2 = rospy.Publisher('path_string_blue_robot', String, queue_size=10)

    while not rospy.is_shutdown():

        coordinate_string = str(current_robot_pose_x) +"," + str(current_robot_pose_y) + "," + str(current_robot_pose_z)

        if should_publish_pose:
            print ("Code comes here")

            # rospy.loginfo(coordinate_string)
            pub.publish(coordinate_string)
            print len(path)


            path_str = str(path)
            path_str = path_str.replace('[','')
            path_str = path_str.replace(']','')
            path_str = path_str.replace('\'','')
            print path_str


            pub2.publish(path_str)
            should_publish_pose = False

        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback)
        rospy.Subscriber("/move_base/NavfnROS/plan", Path, pathcallback)
        talker()
    except rospy.ROSInterruptException:
        pass
