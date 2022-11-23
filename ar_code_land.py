#!/usr/bin/env python2
#***************************************************************************
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#***************************************************************************/

#
# @author Andreas Antener <andreas@uaventure.com>
#
# The shebang of this file is currently Python2 because some
# dependencies such as pymavlink don't play well with Python3 yet.
from __future__ import division

PKG = 'px4'

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import ParamValue
from mavros_test_common import MavrosTestCommon
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler
from ar_track_alvar_msgs.msg import AlvarMarkers

class MavrosOffboardPosctlTest(MavrosTestCommon):
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.
    For the test to be successful it needs to reach all setpoints in a certain time.
    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """

    def setUp(self):
        super(MavrosOffboardPosctlTest, self).setUp()
        self.pos = PoseStamped()
        self.radius = 1
        self.pos_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_position/local', PoseStamped, queue_size=1)
        # send setpoints in separate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()
	self.ar_x = 0
	self.ar_y = 0
	self.ar_flag = False

	self.ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback)


    def callback(self,data):
	if data.markers == []:
	    rospy.loginfo("no marker detect")
	else:
            if data.markers[0].id == 7 and self.ar_flag == False:
                rospy.loginfo("correct AR id: {0}".format(data.markers[0].id))
                x = data.markers[0].pose.pose.position.x	
                y = data.markers[0].pose.pose.position.y		
                z = data.markers[0].pose.pose.position.z		
                rospy.loginfo("x:{0},y:{1},z:{2}".format(x,y,z))
		self.ar_x = y - 0.15
 		self.ar_y = -x
		self.ar_flag = True

    def tearDown(self):
        super(MavrosOffboardPosctlTest, self).tearDown()

    #
    # Helper methods
    #
    def send_pos(self):
        rate = rospy.Rate(10)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset

    def reach_position(self, x, y, z, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z
        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
            format(x, y, z, self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z))

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.pos.pose.position.x,
                                   self.pos.pose.position.y,
                                   self.pos.pose.position.z, self.radius):
                rospy.loginfo("position reached | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                reached = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(reached, (
            "took too long to get to position | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
            format(self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z, timeout)))

    #
    # Test method
    #
    def test_posctl(self):
        """Test offboard position control"""

        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        self.log_topic_vars()
        # exempting failsafe from lost RC to allow offboard
        rcl_except = ParamValue(1<<2, 0.0)
        self.set_param("COM_RCL_EXCEPT", rcl_except, 5)
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)

        rospy.loginfo("run mission")
        ##### Check GPS safty ######

        x0_record = []
        y0_record = []
        rate = rospy.Rate(1)  # 1 Hz

        for count in range(3):
            rate.sleep()  # Sleeps for 1/rate sec
            x0_tmp = self.local_position.pose.position.x
            y0_tmp = self.local_position.pose.position.y
            x0_record.append(x0_tmp)
            y0_record.append(y0_tmp)
            rospy.loginfo("GPS position  | {0} , {1}".format(x0_tmp, y0_tmp))
            rospy.loginfo("==================== Recording Initial GPS Position ====================")


        x0_ave = np.average(x0_record)
        y0_ave = np.average(y0_record)
        x0_var = np.var(x0_record)
        y0_var = np.var(y0_record)
        rospy.loginfo("GPS position AVG | {0} , {1} ".format(x0_ave, y0_ave))
        rospy.loginfo("GPS position VAR | {0} , {1} ".format(x0_var, y0_var))

        if x0_var < 1 and y0_var < 1:
            x0 = x0_tmp # x0_ave
            y0 = y0_tmp # y0_ave

            rospy.loginfo("==================== GPS variance is low, good to fly ====================")
        else:
            rospy.loginfo("GPS variance is too big, Stop and disarm")
            self.set_arm(False, 5)
        ##### Check GPS safty ######

        positions = [[0, 0, 1.5]]

        for update_pos in positions:
            update_pos[0] = update_pos[0] + x0
            update_pos[1] = update_pos[1] + y0

        # positions = ((0, 0, 0), (50, 50, 20), (50, -50, 20), (-50, -50, 20),
        #              (0, 0, 20))

        for i in xrange(len(positions)):
            self.reach_position(positions[i][0], positions[i][1],
                                positions[i][2], 30)

        rospy.loginfo("==================== Reach high point and start to detect AR code ====================")
        detect_max_time = 30
        for count in range(detect_max_time):
            rest_time = detect_max_time-count
            rospy.loginfo("==================== AR code detecting, rest time {0}  second ====================".format(rest_time))
            if self.ar_flag == True:
		rospy.loginfo("==================== Detect AR code, Goalx: {0}, Goaly: {1} ====================".format(self.ar_x,self.ar_y))
		x0 = self.local_position.pose.position.x
		y0 = self.local_position.pose.position.y
		z0 = self.local_position.pose.position.z
		ar_goal = [x0+self.ar_x, y0+self.ar_y, z0]
		if abs(ar_goal[0]) > 5 or abs(ar_goal[1]) > 5:
		    rospy.loginfo("exceed x y limitation, do not move")
		elif abs(ar_goal[2]) > 10:
		    rospy.loginfo("exceed z limitation, do not move")
		else:
		    self.reach_position(ar_goal[0], ar_goal[1],ar_goal[2], 30)
		    rospy.loginfo("==================== Arive AR High Position , {0} ====================".format(ar_goal))
		break
            rate.sleep()  # Sleeps for 1/rate sec

        rospy.loginfo("autoland")
        self.set_mode("AUTO.LAND", 5)
        rospy.loginfo("ready to dis arm")
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   45, 0)
        self.set_arm(False, 5)


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)

    rostest.rosrun(PKG, 'mavros_offboard_posctl_test',
                   MavrosOffboardPosctlTest)
