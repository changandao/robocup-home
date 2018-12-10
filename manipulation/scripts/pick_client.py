#!/usr/bin/env python

# Copyright (c) 2016 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Author:
#   * Sam Pfeiffer
#   * Job van Dieten
#   * Jordi Pages

import rospy
import time
from tiago_pick_demo.msg import PickUpPoseAction, PickUpPoseGoal
from geometry_msgs.msg import PoseStamped, Pose, WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient

from tum_alle_athome_speech_msgs.srv import srvTTS, srvTTSRequest, srvTTSResponse

import tf2_ros
from tf2_geometry_msgs import do_transform_pose

import numpy as np
from std_srvs.srv import Empty
from all_msgs.srv import send_flags, send_flagsRequest
import cv2
from cv_bridge import CvBridge

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
	if not name[:1] == '_':
		code = MoveItErrorCodes.__dict__[name]
		moveit_error_dict[code] = name

class SphericalService(object):
	def __init__(self):
		rospy.loginfo("Starting Spherical Grab Service")
		self.readygo = rospy.Service("/tiago/readygo", Empty, self.start_aruco_ready)

		rospy.loginfo("Finished SphericalService constructor")
                self.place_gui = rospy.Service("/tiago/place", Empty, self.start_aruco_place)
                self.lower_torso = rospy.Service("/tiago/lower_torso", send_flags, self.lower_torso)
                self.pick_gui = rospy.Service("/tiago/pick", Empty, self.start_aruco_pick)
		self.place_start = rospy.Service("/place/start", send_flags, self.start_place)
                #self.force = rospy.Subscriber("/wrist_ft", WrenchStamped, self.force_value)

	def lower_torso(self, req): 
                if req.flag: 
		    	rospy.loginfo("Moving torso down")
		     	jt = JointTrajectory()
		 	jt.joint_names = ['torso_lift_joint']
			jtp = JointTrajectoryPoint()
			jtp.positions = [0.15]
			jtp.time_from_start = rospy.Duration(2.5)
			jt.points.append(jtp)
			self.torso_cmd.publish(jt)
			return 1
		else:
			return 0
						

	def start_aruco_ready(self, req):
		
		self.pick_type = PickAruco()
		#res.reply = 1
		return 1
		


	def start_aruco_pick(self, req):
		
		self.pick_type.pick_aruco("pick")
		#res.reply = 1
		return 1
		


	def start_aruco_place(self, req):
		
		self.pick_type.place_aruco("place")  # change pick to place
		#res.reply = 1
		return 1


	def start_place(self, req):
		if req.flag:
			self.pick_type.place_aruco("place")  # change pick to place
			#res.reply = 1
			return 1
		else: return 0


class PickAruco(object):
	def __init__(self):
		rospy.loginfo("Initalizing...")
                self.bridge = CvBridge()
		self.tfBuffer = tf2_ros.Buffer()
                self.tf_l = tf2_ros.TransformListener(self.tfBuffer)
                 
		rospy.loginfo("Waiting for /pickup_pose AS...")
		self.pick_as = SimpleActionClient('/pickup_pose', PickUpPoseAction)
		
		#self.force = rospy.Subscriber("/wrist_tf", WrenchStamped, self.force_value)
		
		
                time.sleep(1.0)
		if not self.pick_as.wait_for_server(rospy.Duration(20)):
			rospy.logerr("Could not connect to /pickup_pose AS")
			exit()
		rospy.loginfo("Waiting for /place_pose AS...")
		self.place_as = SimpleActionClient('/place_pose', PickUpPoseAction)

		self.place_as.wait_for_server()

		rospy.loginfo("Setting publishers to torso and head controller...")
		self.torso_cmd = rospy.Publisher(
			'/torso_controller/command', JointTrajectory, queue_size=1)
		self.head_cmd = rospy.Publisher(
			'/head_controller/command', JointTrajectory, queue_size=1)
		self.detected_pose_pub = rospy.Publisher('/detected_aruco_pose',
							 PoseStamped,
							 queue_size=1,
							 latch=True)

		rospy.loginfo("Waiting for '/play_motion' AS...")


		self.place_finish = rospy.ServiceProxy(
			'/god_watcher/place_finish', send_flags)

		#self.place_finish = SimpleActionClient('/god_watcher/place_finish', send_flags)

		self.play_m_as = SimpleActionClient('/play_motion', PlayMotionAction)



		self.speak = rospy.ServiceProxy('/tiago/speech/tts', srvTTS)
		self.speak.wait_for_service()
		rospy.loginfo("Speak node Connected.")





		if not self.play_m_as.wait_for_server(rospy.Duration(20)):
			rospy.logerr("Could not connect to /play_motion AS")
			exit()
		rospy.loginfo("Connected!")
		rospy.sleep(1.0)
		rospy.loginfo("Done initializing PickAruco.")
		
		

   	def strip_leading_slash(self, s):
		return s[1:] if s.startswith("/") else s
		
		
	def force_value(self, data):
                force_x = np.absolute(data.wrench.force.x)
                force_y = np.absolute(data.wrench.force.y)
                force_z = np.absolute(data.wrench.force.z)
                force_abs = force_x + force_y + force_z
                rospy.loginfo("I have force_abs")
		if force_abs >= 10.0:
			rospy.loginfo("I got the bag.")	

	def tiago_speak(self, string_text):
        	rospy.loginfo(string_text)
        	msg_speech = srvTTSRequest()
        	msg_speech.text = string_text
		self.speak.call(msg_speech)
        	rospy.sleep(0.1)
        	rospy.loginfo("Done talking.")
		
		
	def pick_aruco(self, string_operation):
		self.prepare_robot()

		rospy.sleep(2.0)
		rospy.loginfo("spherical_grasp_gui: Waiting for an aruco detection")
                #rospy.sleep(5.0)


		aruco_pose = rospy.wait_for_message('/object_detection/Pick_Pose', PoseStamped)
		aruco_pose.header.frame_id = self.strip_leading_slash(aruco_pose.header.frame_id)
		rospy.loginfo("Got: " + str(aruco_pose))


		rospy.loginfo("spherical_grasp_gui: Transforming from frame: " +
		aruco_pose.header.frame_id + " to 'base_footprint'")
		ps = PoseStamped()
		ps.pose.position = aruco_pose.pose.position
		ps.header.stamp = self.tfBuffer.get_latest_common_time("base_footprint", aruco_pose.header.frame_id)
		ps.header.frame_id = aruco_pose.header.frame_id
		transform_ok = False
		while not transform_ok and not rospy.is_shutdown():
			try:
				transform = self.tfBuffer.lookup_transform("base_footprint",
									   ps.header.frame_id,
									   rospy.Time(0))
				aruco_ps = do_transform_pose(ps, transform)
				transform_ok = True
			except tf2_ros.ExtrapolationException as e:
				rospy.logwarn(
					"Exception on transforming point... trying again \n(" +
					str(e) + ")")
				rospy.sleep(0.01)
				ps.header.stamp = self.tfBuffer.get_latest_common_time("base_footprint", aruco_pose.header.frame_id)
			pick_g = PickUpPoseGoal()

		if string_operation == "pick":

                        rospy.loginfo("Setting cube pose based on bottle detection")
			
			pick_g.object_pose.pose.position.x = aruco_ps.pose.position.x + 0.04
			pick_g.object_pose.pose.position.y = aruco_ps.pose.position.y - 0.04
			pick_g.object_pose.pose.position.z = aruco_ps.pose.position.z + 0.14
			# pick_g.object_pose.pose.orientation = aruco_ps.pose.position
                        # pick_g.object_pose.pose.position = aruco_ps.pose.position
                        # pick_g.object_pose.pose.position.z -= 0.1*(1.0/2.0)

                        rospy.loginfo("aruco pose in base_footprint:" + str(pick_g))

			pick_g.object_pose.header.frame_id = 'base_footprint'
			pick_g.object_pose.pose.orientation.w = 1.0
			self.detected_pose_pub.publish(pick_g.object_pose)
			rospy.loginfo("Gonna pick:" + str(pick_g))
			self.pick_as.send_goal_and_wait(pick_g)
			rospy.loginfo("Done!")

			result = self.pick_as.get_result()
			if str(moveit_error_dict[result.error_code]) != "SUCCESS":
				rospy.logerr("Failed to pick, not trying further")
				return

				
			rospy.loginfo("######before")
			# Move torso to its maximum height
                        self.lift_torso()
                        # Raise arm
			#rospy.loginfo("Moving arm to a safe pose")
			pmg = PlayMotionGoal()
                        pmg.motion_name = 'pick_final_pose'
			pmg.skip_planning = False
			self.play_m_as.send_goal_and_wait(pmg)
			data = rospy.wait_for_message('/wrist_ft', WrenchStamped)
	                force_x = np.absolute(data.wrench.force.x)
                	force_y = np.absolute(data.wrench.force.y)
                	force_z = np.absolute(data.wrench.force.z)
                	force_abs = force_x + force_y + force_z
                	rospy.loginfo("I have force_abs")
<<<<<<< HEAD
			if force_abs >= 30.0:
				rospy.loginfo("I got the bag text.")
=======
			if force_abs >= 28.0:
				rospy.loginfo("I got the bag.")
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e
				self.tiago_speak("I got the bag.")
			else:
				rospy.loginfo("where is the bag.")
				self.tiago_speak("I cannot pick the bag, please try again.")
	                #data = rospy.Subscriber('/wrist_ft', WrenchStamped, self.force_value)
	
			rospy.loginfo("Raise object done.")
			rospy.loginfo("#######end")

			


	def place_aruco(self, string_operation):
			rospy.sleep(2.0)
                        #self.prepare_robot()


			if string_operation == "place":

				# pick_g.object_pose.pose.position.z -= 0.1*(1.0/2.0)
				
				
	                        # Place the object back to its position
				rospy.loginfo("Gonna place near where it was")
				pick_g = PickUpPoseGoal()
				pick_g.object_pose.pose.position.x = 0.85
				pick_g.object_pose.pose.position.y = 0.00
				pick_g.object_pose.pose.position.z = 1.00
				# pick_g.object_pose.pose.orientation = aruco_ps.pose.position
                        	# pick_g.object_pose.pose.position = aruco_ps.pose.position
                        	# pick_g.object_pose.pose.position.z -= 0.1*(1.0/2.0)

                        	rospy.loginfo("aruco pose in base_footprint:" + str(pick_g))

				pick_g.object_pose.header.frame_id = 'base_footprint'
				pick_g.object_pose.pose.orientation.w = 1.0
				pick_g.object_pose.pose.position.z += 0.05
				self.place_as.send_goal_and_wait(pick_g)
				rospy.loginfo("Done!")
<<<<<<< HEAD
						        		

				rospy.loginfo("Done!")
				data = rospy.wait_for_message('/wrist_ft', WrenchStamped)
	                	force_x = np.absolute(data.wrench.force.x)
                		force_y = np.absolute(data.wrench.force.y)
                		force_z = np.absolute(data.wrench.force.z)
                		force_abs = force_x + force_y + force_z
                		rospy.loginfo("I have force_abs")
				if force_abs < 35.0:
					rospy.loginfo("I release the bag text.")
					self.tiago_speak("I released the bag.")
				else:
					rospy.loginfo("please hold the bag.")
					self.tiago_speak("Please pick the bag, i can't holding it anymore.")
			pmg = PlayMotionGoal()
                        pmg.motion_name = 'pick_final_pose'
			pmg.skip_planning = False
			self.play_m_as.send_goal_and_wait(pmg)
				
				
			rospy.loginfo("#######end")


				#self.place_finish.wait_for_service()
				#finish_flag = send_flags()
				#finish_flag.Request.flag = 1
				#self.place_finish.call(finish_flag)

				#rospy.loginfo("call speak done.")
=======
				
                                # Raise arm
			        rospy.loginfo("Moving arm to a safe pose")
			        pmg = PlayMotionGoal()
                                pmg.motion_name = 'pick_final_pose'
			        pmg.skip_planning = False
			        self.play_m_as.send_goal_and_wait(pmg)
			        rospy.loginfo("Raise object done.")

				rospy.loginfo("Done!")


				self.place_finish.wait_for_service()
				finish_flag = send_flags()
				finish_flag.Request.flag = 1
				self.place_finish.call(finish_flag)

				rospy.loginfo("call speak done.")
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e



			# Place the object back to its position
		# rospy.loginfo("Gonna place near where it was")
		# pick_g.object_pose.pose.position.z += 0.05
		# self.place_as.send_goal_and_wait(pick_g)
		# rospy.loginfo("Done!")

	def lift_torso(self):
		rospy.loginfo("Moving torso up")
		jt = JointTrajectory()
		jt.joint_names = ['torso_lift_joint']
		jtp = JointTrajectoryPoint()
		jtp.positions = [0.20] #0.34
		jtp.time_from_start = rospy.Duration(2.5)
		jt.points.append(jtp)
		self.torso_cmd.publish(jt)

        def lower_head(self):
		rospy.loginfo("Moving head down")
		jt = JointTrajectory()
		jt.joint_names = ['head_1_joint', 'head_2_joint']
		jtp = JointTrajectoryPoint()
		jtp.positions = [0.0, -0.45] #0.0, -0.75
		jtp.time_from_start = rospy.Duration(2.0)
		jt.points.append(jtp)
		self.head_cmd.publish(jt)
                rospy.loginfo("Done.")

	def prepare_robot(self):
		rospy.loginfo("Unfold arm safely")
		pmg = PlayMotionGoal()
		pmg.motion_name = 'pregrasp'
		pmg.skip_planning = False
		self.play_m_as.send_goal_and_wait(pmg)
		rospy.loginfo("Done.")

                self.lower_head()

		rospy.loginfo("Robot prepared.")


if __name__ == '__main__':
	rospy.init_node('pick_aruco_demo')
	sphere = SphericalService()
	rospy.spin()

