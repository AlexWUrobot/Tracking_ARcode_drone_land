#!/usr/bin/env python
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers

def callback(data):

    if data.markers[0].id == 7:
	rospy.loginfo("correct id: {0}".format(data.markers[0].id))
	x = data.markers[0].pose.pose.position.x	
	y = data.markers[0].pose.pose.position.y		
	z = data.markers[0].pose.pose.position.z		
	rospy.loginfo("x:{0},y:{1},z:{2}".format(x,y,z))

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    # rospy.loginfo("waiting for message")
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
