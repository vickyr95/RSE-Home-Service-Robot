#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time

def movebase_client(target_x, target_y):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = target_x
    goal.target_pose.pose.position.y = target_y
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')

        # Move to pick location
        pick_x = 4.75  
        pick_y = 0.15 
        result = movebase_client(pick_x, pick_y)
        if result:
            rospy.loginfo("Reached pickup zone")
            time.sleep(5)  # Wait for 5 seconds

        # Move to place location
        place_x = 2.5  
        place_y = 0.32
        result = movebase_client(place_x, place_y)
        if result:
            rospy.loginfo("Reached drop off zone")

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
