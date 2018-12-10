#!/usr/bin/env python

"""
hotword-detect.py
    parameters:
        ~hotword_model_tiago - 
        ~hotword_model_follow - 
        ~hotword_model_bring - 
        ~hotword_model_say - 
        ~hotword_model_move - 
        ~sensitivity - 
        ~speech_recognition_srv - speech recog service name
    publications:

    services:

"""
import roslib; roslib.load_manifest('tum_alle_athome_speech_hotword')
import rospy
import os, os.path
import sys
import signal

current_dir = os.path.abspath(os.path.dirname(__file__))

# set path for snowboy and import it
snowboy_path = (os.path.join(current_dir, '..', '3rdparty', 'snowboy'))
sys.path.append(snowboy_path)
import snowboydecoder

from tum_alle_common_msgs.srv import srvSpeechRecognition
from tum_alle_common_msgs.msg import msgHotwordCommand
from tum_alle_common_msgs.srv import srvPlanning

class HotwordRecogniser(object):
    """ Snowboy based hotword recogniser """

    def __init__(self):
        self.tiago_started = False
        self.commands_started = False
        rospy.init_node("HotwordRecogniser")

        # Get params from .launch file
        self._hotword_tiago_param = "~hotword_model_tiago"
        self._hotword_move_param = "~hotword_model_move"
        self._hotword_say_param = "~hotword_model_say"
        self._hotword_bring_param = "~hotword_model_bring"
        self._hotword_follow_param = "~hotword_model_follow"
        self._hotword_start_param = "~hotword_model_start"
        self._hotword_wait_param = "~hotword_model_wait"
	self._hotword_stop_param = "~hotword_model_stop"
	self._hotword_yes_param = "~hotword_model_yes"
	self._hotword_no_param = "~hotword_model_no"
        self._commands_sensitivity_param = "~commands_sensitivity"
        self._tiago_sensitivity_param = "~tiago_sensitivity"
        self._speech_recognition_srv_param = "~speech_recognition_srv"

        self.cmd_models = []
        # Should in the end contain the number of commands in cmd_models[]
        self.number_of_commands = 1

        self.init_ros()
        self.init_snowboy()
        rospy.loginfo("Hotword recogniser parameters set, starting...")
        self.start_recogniser()

    def init_snowboy(self):
        if rospy.has_param(self._hotword_tiago_param):
            #self.cmd_models.append(rospy.get_param(self._hotword_tiago_param))
            self.hotword_tiago = rospy.get_param(self._hotword_tiago_param)
        else:
            rospy.logerr("No Tiago hotword model defined in the launch file")
            return

        #if rospy.has_param(self._hotword_move_param):
            #self.hotword_move = rospy.get_param(self._hotword_move_param)
            #self.cmd_models.append(rospy.get_param(self._hotword_move_param))
        if rospy.has_param(self._hotword_say_param):
            #self.hotword_say = rospy.get_param(self._hotword_say_param)
            self.cmd_models.append(rospy.get_param(self._hotword_say_param))
        #if rospy.has_param(self._hotword_follow_param):
            #self.hotword_follow = rospy.get_param(self._hotword_follow_param)
            #self.cmd_models.append(rospy.get_param(self._hotword_follow_param))
        if rospy.has_param(self._hotword_bring_param):
            #self.hotword_bring = rospy.get_param(self._hotword_bring_param)
            self.cmd_models.append(rospy.get_param(self._hotword_bring_param))
        if rospy.has_param(self._hotword_start_param):
            #self.hotword_start = rospy.get_param(self._hotword_start_param)
            self.cmd_models.append(rospy.get_param(self._hotword_start_param))
        if rospy.has_param(self._hotword_wait_param):
            #self.hotword_wait = rospy.get_param(self._hotword_wait_param)
            self.cmd_models.append(rospy.get_param(self._hotword_wait_param))
        if rospy.has_param(self._hotword_stop_param):
            #self.hotword_stop = rospy.get_param(self._hotword_stop_param)
            self.cmd_models.append(rospy.get_param(self._hotword_stop_param))
        if rospy.has_param(self._hotword_yes_param):
            #self.hotword_stop = rospy.get_param(self._hotword_stop_param)
            self.cmd_models.append(rospy.get_param(self._hotword_yes_param))
        if rospy.has_param(self._hotword_no_param):
            #self.hotword_stop = rospy.get_param(self._hotword_stop_param)
            self.cmd_models.append(rospy.get_param(self._hotword_no_param))

        # Important: order needs to be the same to the hotwords defined in
        # the list self.cmd_models() defined above
        self.commands_callbacks = [
                #lambda: self.hotword_commands_detected("move"), 
                lambda: self.hotword_commands_detected("say"),
                #lambda: self.hotword_commands_detected("follow"),
                lambda: self.hotword_commands_detected("bring"),
                lambda: self.hotword_commands_detected("start"),
                lambda: self.hotword_commands_detected("wait"),
		lambda: self.hotword_commands_detected("stop"),
		lambda: self.hotword_commands_detected("yes"),
		lambda: self.hotword_commands_detected("no")
            ]

        # Set sensitivity for the hotwords, sensitivity should be passed as list
        # With the sensitivity being assigned to the respective command
        self.number_of_commands = len(self.cmd_models)
        if rospy.has_param(self._commands_sensitivity_param):
            self.commands_sensitivity = (self.number_of_commands * 
                            [rospy.get_param(self._commands_sensitivity_param)])
        else:
            self.commands_sensitivity = self.number_of_commands * [0.5]

        if rospy.has_param(self._tiago_sensitivity_param):
            self.tiago_sensitivity = rospy.get_param(
                                            self._tiago_sensitivity_param)
        else:
            self.tiago_sensitivity = 0.5

        if rospy.has_param(self._speech_recognition_srv_param):
            self.speech_recognition_srv = rospy.get_param(
                                self._speech_recognition_srv_param)
        else:
            #self.speech_recognition_srv = "/tum_alle_athome_speech_recognition" +\
                                                #"/speech_recog"
            self.speech_recognition_srv = "/planner"

        # Callback values indicating if decoder should be interrupted, if True
        # then the hotword detector will be interrupted until started again
        self.interrupt_tiago = False
        self.interrupt_commands = False
        
        rospy.loginfo("Creating tiago detector")
        self.detector_tiago = snowboydecoder.HotwordDetector(
                    self.hotword_tiago, sensitivity=self.tiago_sensitivity)
        rospy.loginfo("Creating commands detector")
        self.detector_commands = snowboydecoder.HotwordDetector(
                    self.cmd_models, sensitivity=self.commands_sensitivity)
        rospy.loginfo("Created commands detector")

    def init_ros(self):
        self.command_pub = rospy.Publisher(
                            'hotword_command',
                            msgHotwordCommand,
                            queue_size=3
                            )
        rospy.on_shutdown(self.stop_recogniser)

    def start_recogniser(self):
        rospy.loginfo("Starting tiago recogniser")
        self.detector_tiago.start(
                    sleep_time=0.03, 
                    detected_callback=self.hotword_tiago_detected,
                    interrupt_check=self.interrupt_tiago_cb)
        self.tiago_started = True
        rospy.loginfo('Tiago recogniser started')

    def hotword_tiago_detected(self):
        """ Called every time when the hotword Tiago is detected"""
        self.interrupt_tiago = True
        self.tiago_started = False
        rospy.loginfo("Hotword: Tiago detected")
        self.detector_commands.start(
            sleep_time=0.03, 
            detected_callback=self.commands_callbacks,
            interrupt_check=self.interrupt_commands_cb)
        self.commands_started = True
        self.interrupt_commands = False
        # Start speech recogniser
        #self.speech_recognition_client("start", "")

    def hotword_commands_detected(self, command):
        """ Called every time when a command hotword is detected"""
        self.interrupt_commands = True
        self.commands_started = False
        print("Detected cmd: " + command)
        self.command_pub.publish(command)

        # Start/wait don't require further speech recog
        if "hotword" != command and "watch" != command:
            self.speech_recognition_client(command,"s","s","s","s","s")

        self.detector_tiago.start(
            sleep_time=0.03, 
            detected_callback=self.hotword_tiago_detected,
            interrupt_check=self.interrupt_tiago_cb)
        self.tiago_started = True
        self.interrupt_tiago = False

    def interrupt_tiago_cb(self):
        tmp = self.interrupt_tiago
        self.interrupt_tiago = False
        return tmp

    def interrupt_commands_cb(self):
        tmp = self.interrupt_commands
        self.interrupt_commands = False
        return tmp

    def speech_recognition_client(self, command, object_general_location, object_specific_location, goal_location, goal_object, speech_text):
        #rospy.wait_for_service(self.speech_recognition_srv + action)
        try:
            speech_recognition = rospy.ServiceProxy(
		self.speech_recognition_srv, srvPlanning)
                #self.speech_recognition_srv, srvSpeechRecognition)

            response = speech_recognition(command, "s","s","s","s","s")
        except rospy.ServiceException, e:
            print("Speech recog " + command + " service call failed: %s" %e)

    def stop_recogniser(self):
        if self.tiago_started is True:
            self.detector_tiago.terminate()
            self.tiago_started = False
        if self.commands_started is True:
            self.detector_commands.terminate()
            self.commands_started = False
        
        # Clean up params for next launch
        for param in [
                self._tiago_sensitivity_param, 
                self._commands_sensitivity_param, self._hotword_tiago_param, 
                self._hotword_bring_param, 
                self._speech_recognition_srv_param, 
<<<<<<< HEAD
                self._hotword_say_param, self._hotword_follow_param,
=======
                self._hotword_say_param,
>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e
                self._hotword_start_param, self._hotword_wait_param,
                self._hotword_stop_param, self._hotword_yes_param, self._hotword_no_param
                    ]:
            if rospy.has_param(param):
                rospy.delete_param(param)


if __name__ == "__main__":
    start = HotwordRecogniser()
