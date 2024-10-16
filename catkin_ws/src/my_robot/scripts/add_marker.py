#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker

def publish_marker():
    rospy.init_node('add_markers')
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

    rospy.sleep(1)  # Wait for the publisher to be ready

    marker = Marker()
    marker.header.frame_id = "map"
    marker.ns = "add_markers"
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration(0)  # Infinite lifetime

    # Publish marker at pick-up zone
    marker.pose.position.x = 4.75  
    marker.pose.position.y = 0.15  
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # Publish the marker
    marker_pub.publish(marker)
    rospy.loginfo("Marker added")
    rospy.sleep(5)

    # Delete the marker
    marker.action = Marker.DELETE
    marker_pub.publish(marker)
    rospy.loginfo("Marker removed")
    rospy.sleep(5)

    # Publish marker at drop-off zone
    marker.action = Marker.ADD
    marker.pose.position.x = 2.5
    marker.pose.position.y = 0.32
    marker.pose.position.z = 0

    # Publish the marker
    marker_pub.publish(marker)
    rospy.loginfo("Marker added")
    rospy.sleep(2)

if __name__ == '__main__':
    try:
        publish_marker()
    except rospy.ROSInterruptException:
        pass
