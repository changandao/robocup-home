#!/usr/bin/env python

import roslaunch
import rospy
import smach
import smach_ros
import numpy as np
from std_srvs.srv import Empty
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from tum_alle_athome_speech_msgs.srv import srvTTS, srvTTSRequest
from tum_alle_common_msgs.msg import msgHotwordCommand
from tum_alle_common_msgs.srv import srvPlanning, srvPlanningRequest, srvPlanningResponse
from all_msgs.msg import *
from all_msgs.srv import send_flagsRequest, send_flags
from all_msgs.srv import send_posRequest


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


class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['started'],
                             input_keys=['desired_pose'],
                             output_keys=['next_desired_pose']
                             )
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid,["/home/atHomeSS18/God-Watcher/workspace/Finalproject/src/god_states_machine/launch/states_launch/speech_manipulation.launch"])

    def execute(self, userdata):
        self.launch.start()
        rospy.sleep(duration=10.0)
        rospy.loginfo('God is coming now')
        speak_client('God is coming now')
        return 'started'


class SpeechRecognition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ready', 'waitcommand', 'localized', 'relocalizing', 'keepfollow', 'bring_bag'],
                             input_keys=['desired_pose'],
                             output_keys=['next_desired_pose']
                             )
        self.command_msg = msgHotwordCommand()
        self.command = ''

        self.speech_count = 0

    def execute(self, ud):
        self.speech_count += 1
        self.command_msg = rospy.wait_for_message("/hotword_command", msgHotwordCommand)
        self.command = self.command_msg.command
        # self.command = raw_input()
        if(self.command == 'say'):
            return 'ready'
        elif (self.command == 'yes'):
            return 'localized'
        elif (self.command == 'no'):
            return 'relocalizing'
        elif (self.command == 'follow' and self.speech_count > 4):
            return 'keepfollow'
        elif (self.command == 'bring'):
            return 'bring_bag'
        else:
            speak_client('I can not hear you clearly')
            return 'waitcommand'
        # if (self.command == 's'):
        #     return 'ready'
        # elif (self.command == 'y'):
        #     return 'localized'
        # elif (self.command == 'n'):
        #     return 'relocalizing'
        # elif (self.command == 'f' and self.speech_count > 4):
        #     return 'keepfollow'
        # elif (self.command == 'b'):
        #     return 'bring_bag'
        # else:
        #     speak_client('I can not hear you clearly')
        #     return 'waitcommand'





class ReadyPick(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pick_done'],
                             input_keys=['desired_pose'],
                             output_keys=['next_desired_pose']
                             )
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/atHomeSS18/God-Watcher/workspace/Finalproject/src/god_states_machine/launch/states_launch/rviz_manipulation.launch"])

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

    def pick_client(self):
        rospy.wait_for_service('/tiago/pick')
        try:
            object_dectection = rospy.ServiceProxy('/tiago/pick', Empty)
            object_dectection()
        except rospy.ServiceException, e:
            print "service call failed: %s" % e

    def execute(self, userdata):
        rospy.loginfo('ready to pick')
        self.launch.start()
        speak_client('Tiago is ready to pick')
        self.object_dectection_client()
        self.readyToPick_client()
        self.pick_client()
        self.launch.shutdown()
        return 'pick_done'


class FollowTop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['followme'],
                             input_keys=['desired_pose'],
                             output_keys=['next_desired_pose']
                             )
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/atHomeSS18/God-Watcher/workspace/Finalproject/src/god_states_machine/launch/states_launch/learn_track_prepare.launch"])

    def execute(self, userdata):
        rospy.loginfo('Now I will follow the person')
        self.launch.start()
        speak_client('I have got the bag')
        return 'followme'


# define state learn_operator
class Localization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['localizing'],
                             input_keys=['desired_pose'],
                             output_keys=['next_desired_pose']
                             )

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
        self.localization_client()
        speak_client("Have I successfully localized")
        return 'localizing'


class Clearcostmap(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['cleared'],
                             input_keys=['desired_pose'],
                             output_keys=['next_desired_pose']
                             )

    def clear_cost_map_client(self):
        rospy.wait_for_service('/move_base/clear_costmaps')
        try:
            clear_cost_map = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            clear_cost_map()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def execute(self, ud):
        speak_client("locolization done")
        self.clear_cost_map_client()
        return 'cleared'


# define state Bar
class Learn_operator(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['learning_done'],
                             input_keys=['desired_pose'],
                             output_keys=['next_desired_pose']
                             )
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [
            "/home/atHomeSS18/God-Watcher/workspace/Finalproject/src/god_states_machine/launch/states_launch/learn.launch"])
        self.collected = String()
        self.collect_service = rospy.Service("/online_collecting/finish", Empty, self.collect)
        self.collect_finish = False

    def collect(self, req):
        self.collect_finish = True
        return 1

    def collect_client(self):
        rospy.wait_for_service('/online_collecting/start')
        try:
            collect = rospy.ServiceProxy('/online_collecting/start', Empty)
            collect()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def learn_client(self):
        rospy.wait_for_service('/online_learning/start')
        try:
            learn = rospy.ServiceProxy('/online_learning/start', Empty)
            learn()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def head_control_client(self):
        rospy.wait_for_service('/head_tracking/control')
        try:
            head_control = rospy.ServiceProxy('/head_tracking/control', Empty)
            head_control()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def execute(self, userdata):
        rospy.loginfo('Executing state operater learn')
        self.launch.start()
        speak_client('I am starting to learn please stand before me')
        # self.head_control_client()
        rospy.sleep(duration=5)
        speak_client('three')
        speak_client('two')
        speak_client('one')
        speak_client('Data collection begin')
        #self.collect_client()
        #self.collected = rospy.wait_for_message('/online_collecting/finish', String)
        # rospy.sleep(duration=30)
        # while True:
        #     if self.collect_finish:
        #         break
        #     else:
        #         continue
        speak_client('data collection complete')
        self.learn_client()
        speak_client('learning complete')
        self.launch.shutdown()
        return 'learning_done'


class Show_learn(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['learnshown'],
                             input_keys=['desired_pose'],
                             output_keys=['next_desired_pose']
                             )
        # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(uuid)
        # self.launch = roslaunch.parent.ROSLaunchParent(uuid, [
        #     "/home/atHomeSS18/God-Watcher/workspace/Finalproject/src/god_states_machine/launch/states_launch/learn.launch"])
        self.collected = String()

    def show_learn_client(self):
        rospy.wait_for_service('/head_tracking/control')
        try:
            head_track = rospy.ServiceProxy('/head_tracking/control', Empty)
            head_track()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def execute(self, userdata):
        rospy.loginfo('Executing state operater learn')
        speak_client('I am about to know my master')
        # self.launch.shutdown()
        return 'learnshown'


# define state follow
class StartFollow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ready_follow'],
                             input_keys=['desired_pose'],
                             output_keys=['next_desired_pose'])
        rospy.Subscriber("/navigation/tracking_point", PointStamped, self.getpoint)
        self.desired_pose = send_posRequest()
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/atHomeSS18/God-Watcher/workspace/Finalproject/src/god_states_machine/launch/states_launch/track.launch"])

    def getpoint(self, point_data):
        self.desired_pose.world_pose = point_data
        self.desired_pose.is_target = True

    def ukf_head_control_client(self):
        rospy.wait_for_service('/ukf/head_tracking/control')
        try:
            ukf_head_control = rospy.ServiceProxy('/ukf/head_tracking/control', Empty)
            ukf_head_control()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def execute(self, userdata):
        rospy.loginfo('Executing state follow')
        self.launch.start()
        self.ukf_head_control_client()
        #userdata.next_desired_pose = self.desired_pose

        speak_client('Now I am going to follow')
        speak_client("ready to follow")
        return 'ready_follow'


class Followme(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['following', 'wrong', 'done'],
                             input_keys=['desired_pose'],
                             output_keys=['next_desired_pose'])
        #rospy.Subscriber("/navigation/tracking_point", PointStamped, self.getpoint)
        # rospy.Subscriber("/hotword_command", msgHotwordCommand, self.getcommand)
        self.follow_flags = send_flagsRequest()
        self.onpose = 0
        self.end_signal = 'yes'
        self.follow_count = 0

    def getcommand(self, command_data):
        self.end_signal = command_data.command

    def getpoint(self, point_data):
        self.desired_pose.world_pose = point_data
        self.desired_pose.is_target = True

    def follow_client(self, flags):
        rospy.wait_for_service('/navigation/follow')
        try:
            learn = rospy.ServiceProxy('/navigation/follow', send_flags)
            learnResponse = learn(flags)
            #learnResponse = learn
            return learnResponse.reply
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def execute(self, userdata):
        rospy.loginfo('Executing state follow')
        self.follow_flags.flag = 1
        #while True:
        self.onpose = self.follow_client(1)
        rospy.loginfo(self.onpose)
        self.follow_count += 1
        if(self.onpose == 0):
            rospy.loginfo('wrong')
            return 'wrong'
        else:
            if(self.follow_count >20 or self.end_signal == 'bring'):
                rospy.loginfo('done')
                return 'done'
            else:
                speak_client('following')
                rospy.loginfo('Executing state follow')
                return 'following'



class Iflose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['lose', 'notlose'],
                             input_keys=['desired_pose'],
                             output_keys=['next_desired_pose']
                             )
        self.iflose = rospy.Service("/ukf/refind_target", Empty, self.refind_target)
        self.islose = False

    def refind_target(self, req):
        self.islose = True
        return True

    def execute(self, ud):
        if self.islose:
            speak_client('I lose the master please show yourself again')
            self.islose = False
            return 'lose'
        else:
            return 'notlose'


class Givebag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['baggiven'],
                             input_keys=['desired_pose'],
                             output_keys=['next_desired_pose']
                             )

    def bring_bag_client(self):
        rospy.wait_for_service('/tiago/place')
        try:
            bring_bag = rospy.ServiceProxy('/tiago/place', Empty)
            bring_bag()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def execute(self, ud):
        rospy.loginfo('God is going away')
        self.bring_bag_client()
        speak_client('Bag has been given')
        return 'baggiven'


class End(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['end'],
                             input_keys=['desired_pose'],
                             output_keys=['next_desired_pose']
                             )

    def execute(self, ud):
        rospy.loginfo('God is going away')
        speak_client('God has gone, do not miss him')
