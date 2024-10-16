#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker
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

def publish_marker(marker_pub, action, position):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.ns = "add_markers"
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = action
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # Publish the marker
    marker_pub.publish(marker)

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_with_markers')

        marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        rospy.sleep(1) 

        # pickup and drop-off locations
        pickup_position = (4.75, 0.15)
        dropoff_position = (2.5, 0.32)

        # Publish marker at pick-up zone
        publish_marker(marker_pub, Marker.ADD, pickup_position)
        rospy.loginfo("object available at pickup zone")

        # Move to pickup zone
        result = movebase_client(pickup_position[0], pickup_position[1])
        if result:
            rospy.loginfo("Reached pickup zone")
            publish_marker(marker_pub, Marker.DELETE, pickup_position)
            rospy.loginfo("Object pickuped up")
            time.sleep(5)  # Wait for 5 seconds to simulate pickup

        # Move to drop-off location
        result = movebase_client(dropoff_position[0], dropoff_position[1])
        if result:
            rospy.loginfo("Reached drop-off zone")
            time.sleep(5) # Wait for 5 seconds to simulate drop
        
        # Show marker at drop-off zone
        publish_marker(marker_pub, Marker.ADD, dropoff_position)
        rospy.loginfo("Object dropped at drop-off zone")

        # Move to drop-off location
        result = movebase_client(dropoff_position[0], dropoff_position[1]+0.5)
        if result:
            rospy.loginfo("Reached drop-off zone")
            time.sleep(1) # Wait for 5 seconds to simulate drop

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
