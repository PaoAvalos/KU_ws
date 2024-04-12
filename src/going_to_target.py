#!/usr/bin/python3

import rospy
import actionlib
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
import time 

def move_to_goal(goal_id):

    global mode
    goal = MoveBaseGoal()
    rospy.loginfo("started goal")
    goal.target_pose.header.frame_id = "map"

    # Need to define the goals basen on points from the simulation - do $ rostopic echo /move_base_simple/goal to get the goal points
    if goal_id == "goal1":
        x_goal=  2.7837772369384766
        y_goal= 5.369137287139893
        z_goal= -0.992156449789942
        w_goal= 0.12500231653940738
        
    goal.target_pose.pose.position.x = x_goal
    goal.target_pose.pose.position.y = y_goal
    goal.target_pose.pose.orientation.z = z_goal
    goal.target_pose.pose.orientation.w = w_goal

    rospy.loginfo("defined goal")


    # Create a SimpleActionClient for MoveBase
    move_base_client = SimpleActionClient("move_base", MoveBaseAction)

    # Send the goal to MoveBase and wait for completion
    move_base_client.wait_for_server()
    move_base_client.send_goal(goal)
    rospy.loginfo("sent goal")
    time1= time.time()
    move_base_client.wait_for_result() #here should wait until the movebase is finished
    time_end = time.time()
    # Check if the navigation was successful
    if move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("success")
        time_taken= time_end - time1
        rospy.loginfo("Time to reach goal: %s", time_taken)
    else:
        rospy.logwarn("not successful")

    
def main():
    global success
    rospy.init_node('main_node')
        
    move_to_goal("goal1") #this needs to change when you get to the goal     
    rospy.spin()

if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        rospy.loginfo("code finished")
