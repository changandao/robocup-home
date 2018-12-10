#!/usr/bin/env python

import roslaunch
import rospy
import smach
import smach_ros
import numpy as np
from std_srvs.srv import Empty
from geometry_msgs.msg import PointStamped
#from std_msgs.msg import String
from tum_alle_athome_speech_msgs.srv import srvTTS, srvTTSRequest
<<<<<<< HEAD
from tum_alle_common_msgs.msg import msgHotwordCommand
=======
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e
from tum_alle_common_msgs.srv import srvPlanning, srvPlanningRequest, srvPlanningResponse
from all_msgs.msg import *
from all_msgs.srv import send_flags
from all_msgs.srv import send_pos


<<<<<<< HEAD
pickcount = 0


def speak_client(text):
    speech_msg = srvTTSRequest()
    speech_msg.text = text
    rospy.wait_for_service('/tiago/speech/tts')
    try:
        speak = rospy.ServiceProxy('/tiago/speech/tts', srvTTS)
        speakResponse = speak(speech_msg)
        #return speakResponse.result
    except rospy.ServiceException, e:
        print "service call failed: %s" % e


=======
def speak_client(text):
    speech_msg = srvTTSRequest()
    speech_msg.text = text
    rospy.wait_for_service('/tiago/speech/tts')
    try:
        speak = rospy.ServiceProxy('/tiago/speech/tts', srvTTS)
        speakResponse = speak(speech_msg)
        #return speakResponse.result
    except rospy.ServiceException, e:
        print "service call failed: %s" % e


>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e
class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['started'])

    def execute(self, userdata):
<<<<<<< HEAD
        #rospy.sleep(duration=10.0)
=======
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e
        rospy.loginfo('God is coming now')
        speak_client('God is coming now')
        return 'started'


class SpeechRecognition(smach.State):
    def __init__(self):
<<<<<<< HEAD
        smach.State.__init__(self, outcomes=['ready', 'waitcommand', 'localized', 'relocalizing', 'keepfollow', 'bring_bag'])
        #rospy.Subscriber("hotword_command", String, self.getcommand)
        self.command_msg = msgHotwordCommand()
        self.command = ''

    def execute(self, ud):
        self.command_msg = rospy.wait_for_message("/hotword_command", msgHotwordCommand)
        self.command = self.command_msg.command
        if(self.command == 'say'):
            return 'ready'
=======
        smach.State.__init__(self, outcomes=['waitcommand', 'getbag', 'notgetbag', 'localized', 'relocalizing', 'keepfollow', 'bring_bag'])
        #rospy.Subscriber("hotword_command", String, self.getcommand)
        rospy.Service("/planner", srvPlanning, self.getcommand)
        self.command = 'empty'

    def getcommand(self, command_data):
        self.command = command_data.command

    def execute(self, ud):
        if(self.command == 'empty'):
            return 'waitcommand'
        elif(self.command == 'wait'):
            return 'getbag'
        elif(self.command == 'start'):
            return 'notgetbag'
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e
        elif (self.command == 'yes'):
            return 'localized'
        elif (self.command == 'no'):
            return 'relocalizing'
        elif (self.command == 'follow'):
            return 'keepfollow'
        elif (self.command == 'bring'):
            return 'bring_bag'
<<<<<<< HEAD
        else:
            speak_client('I can not hear you clearly')
            return 'waitcommand'
=======
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e


class ReadyPick(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ready_pick'])

    def object_dectection_client(self):
        rospy.wait_for_service('/tiago/pick_object')
        try:
            object_dectection = rospy.ServiceProxy('/tiago/pick_object', Empty)
            object_dectection()
        except rospy.ServiceException, e:
            print "service call failed: %s" % e

    def readyToPick_client(self):
        rospy.wait_for_service('/tiago/readygo')
        try:
            readyToPick = rospy.ServiceProxy('/tiago/readygo', Empty)
            readyToPick()
        except rospy.ServiceException, e:
            print "service call failed: %s" % e

    def execute(self, userdata):
        rospy.loginfo('ready to pick')
<<<<<<< HEAD
        #self.object_dectection_client()
        #self.readyToPick_client()
=======
        self.object_dectection_client()
        self.readyToPick_client()
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e
        speak_client('Tiago is ready to pick')
        return 'ready_pick'


class Pickbag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pick_done'])

<<<<<<< HEAD
    def pick_client(self):
=======
    def Pick_client(self):
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e
        rospy.wait_for_service('/tiago/pick')
        try:
            object_dectection = rospy.ServiceProxy('/tiago/pick', Empty)
            object_dectection()
        except rospy.ServiceException, e:
            print "service call failed: %s" % e

<<<<<<< HEAD
    def execute(self, ud):
        rospy.loginfo('now I am picking')
        #self.pick_client()
        speak_client('Tiago has picked the bag')
=======
    def excute(self, userdata):
        rospy.loginfo('now I am picking')
        self.Pick_client()
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e
        return 'pick_done'


class FollowTop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['followme'])
        self.confirm = "something"

    def execute(self, userdata):
        rospy.loginfo('Now I will follow the person')
<<<<<<< HEAD
        speak_client('I have got the bag and master please show')
        return 'followme'

=======
        speak_client('I have got the bag you want')
        return 'followme'
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e

# define state learn_operator
class Localization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['localizing'])
        #speak_client('Now I am going to localize')
        self.confirm = "something"

    def localization_client(self):
        rospy.wait_for_service('/god_watcher/loc')
        try:
            localization = rospy.ServiceProxy('/god_watcher/loc', Empty)
            localization()
        except rospy.ServiceException, e:
            print "service call failed: %s" % e

    def execute(self, userdata):
        rospy.loginfo('starting the localization')
        speak_client("I am going to do localization, please leave me alone")
<<<<<<< HEAD
        #self.localization_client()
=======
        self.localization_client()
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e
        return 'localizing'


class Clearcostmap(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['cleared'])

    def clear_cost_map_client(self):
        rospy.wait_for_service('/move_base/clear_costmaps')
        try:
            clear_cost_map = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            clear_cost_map()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def execute(self, ud):
<<<<<<< HEAD
        speak_client("locolization done")
        #self.clear_cost_map_client()
        return 'cleared'

=======
        self.clear_cost_map_client()
        return 'cleared'
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e

# define state Bar
class Learn_operator(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['learning_done'])
        #rospy.Service()

    def collect_client(self):
        rospy.wait_for_service('/online_collecting/start')
        try:
            collect = rospy.ServiceProxy('/online_collecting/start', send_flags)
            collect()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def learn_client(self):
        rospy.wait_for_service('/online_learning/start')
        try:
            learn = rospy.ServiceProxy('/online_learning/start', send_flags)
            learn()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def execute(self, userdata):
<<<<<<< HEAD
        rospy.loginfo('Executing state operater learn')
        speak_client('I am starting to learn please stand before me')
        # self.collect_client()
        # self.learn_client()
=======
        rospy.loginfo('Executing state BAR')
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e
        return 'learning_done'


# define state follow
class StartFollow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ready_follow'],
                             input_keys=[''],
                             output_keys=['next_desired_pose'])
        rospy.Subscriber("/navigation/tracking_point", PointStamped, self.getpoint)
        #self.point = PointStamped()
        self.desired_pose = send_pos()

    def getpoint(self, point_data):
        #self.point = point_data
        self.desired_pose.world_pose = point_data
        self.desired_pose.is_target = True

    def head_track_client(self):
        rospy.wait_for_service('/head_tracking/control')
        try:
            head_track = rospy.ServiceProxy('/head_tracking/control', Empty)
            head_track()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def ukf_head_control_client(self):
        rospy.wait_for_service('/ukf/head_tracking/control')
        try:
            ukf_head_control = rospy.ServiceProxy('/ukf/head_tracking/control', Empty)
            ukf_head_control()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def execute(self, userdata):
        rospy.loginfo('Executing state follow')
<<<<<<< HEAD
        #self.ukf_head_control_client()
        #self.head_track_client()
        #userdata.next_desired_pose = self.desired_pose
        #speak_client('Now I am going to follow')
        speak_client("ready to follow")
        return 'ready_follow'


class Followme(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['following', 'wrong', 'done'],
                             input_keys=['desired_pose'],
                             output_keys=['next_desired_pose'])
        rospy.Subscriber("/navigation/tracking_point", PointStamped, self.getpoint)
        rospy.Subscriber("/hotword_command", msgHotwordCommand, self.getcommand)
        #self.point = PointStamped()
        self.desired_pose = send_pos()
        self.onpose = 0
        self.end_signal = ''

    def getcommand(self, command_data):
        self.end_signal = command_data.command

    def getpoint(self, point_data):
        self.desired_pose.world_pose = point_data
        self.desired_pose.is_target = True

    def follow_client(self, deseired_point):
        rospy.wait_for_service('/navigation/follow')
        try:
            learn = rospy.ServiceProxy('/navigation/follow', send_pos)
            learnResponse = learn(deseired_point)
            return learnResponse.flag
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def execute(self, userdata):
        rospy.loginfo('Executing state follow')
        speak_client("following")
        #self.onpose = self.follow_client(userdata.desired_pose)
        self.onpose += 1
        if(self.onpose != 0 and (self.end_signal == '')):
            userdata.next_desired_pose = self.desired_pose
            return 'following'
        elif(self.onpose == 0 and (self.end_signal == '')):
            speak_client('Sorry master, I fail to follow you')
            return 'wrong'
        elif(self.onpose != 0 and (self.end_signal == 'bring')):
            return 'done'
        else:
            rospy.loginfo(self.onpose)
            rospy.loginfo(self.end_signal)
            return 'following'


=======
        self.ukf_head_control_client()
        self.head_track_client()
        userdata.next_desired_pose = self.desired_pose
        #speak_client('Now I am going to follow')
        return 'ready_follow'


class Followme(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['following', 'wrong', 'done'],
                             input_keys=['desired_pose'],
                             output_keys=['next_desired_pose'])
        rospy.Subscriber("/navigation/tracking_point", PointStamped, self.getpoint)
        #self.point = PointStamped()
        self.desired_pose = send_pos()
        self.onpose = 0
        self.end_signal = True

    def getpoint(self, point_data):
        self.desired_pose.world_pose = point_data
        self.desired_pose.is_target = True

    def follow_client(self, deseired_point):
        rospy.wait_for_service('/navigation/follow')
        try:
            learn = rospy.ServiceProxy('/navigation/follow', send_pos)
            learnResponse = learn(deseired_point)
            return learnResponse.flag
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def execute(self, userdata):
        rospy.loginfo('Executing state follow')
        self.onpose = self.follow_client(userdata.desired_pose)
        if(self.onpose == 1 and not(self.end_signal)):
            userdata.next_desired_pose = self.desired_pose
            return 'following'
        elif(self.onpose != 1 and not(self.end_signal)):
            speak_client('Sorry master, I fail to follow you')
            return 'wrong'
        else:
            return 'done'


>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e
class Givebag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['baggiven'])

<<<<<<< HEAD
    def bring_bag_client(self):
        rospy.wait_for_service('/tiago/place')
        try:
            bring_bag = rospy.ServiceProxy('/tiago/place', Empty)
=======
    def bring_bag_client(self, deseired_point):
        rospy.wait_for_service('/tiago/place')
        try:
            bring_bag = rospy.ServiceProxy('/tiago/place', send_pos)
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e
            bring_bag()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def execute(self, ud):
        rospy.loginfo('God is going away')
<<<<<<< HEAD
        #self.bring_bag_client()
        speak_client('Bag has been given')
=======
        self.bring_bag_client()
        speak_client('God has gone, do not miss him')
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e


class End(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['end'])

    def execute(self, ud):
        rospy.loginfo('God is going away')
        speak_client('God has gone, do not miss him')
