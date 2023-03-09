#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped


current_robot_pose_x = 0
current_robot_pose_y = 0
current_robot_pose_z = 0


# def callback(data):
#     global current_robot_pose_x, current_robot_pose_y, current_robot_pose_z
#     current_robot_pose_x = data.pose.pose.position.x
#     current_robot_pose_y = data.pose.pose.position.y
#     current_robot_pose_z = data.pose.pose.position.z


def check_current_table(data):
    print "Current table is: "+str(data)

def talker():

    pub = rospy.Publisher('pose_string_red_robot', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        object_to_project = "blue_1"
        projectARObject.publish(object_to_project)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('test_gaze_publish', anonymous=True)
        rate = rospy.Rate(10) # 1hz
        projectARObject = rospy.Publisher('object_to_project', String, queue_size=10)
        rospy.Subscriber("/gaze_table", String, check_current_table)
        talker()
    except rospy.ROSInterruptException:
        pass
