#!/usr/bin/env python
import sys
import rospy
import os
import string
from actionlib import SimpleActionClient
from std_msgs.msg import String
from tum_alle_athome_speech_msgs.srv import srvTTS
from tum_alle_athome_speech_msgs.msg import msgTTSState
from pal_interaction_msgs.msg import TtsAction, TtsGoal, TtsActionGoal, I18nText,I18nArgument

## Class storing basic TTS methods
#
class TextToSpeech:

    ## Create a Text-To-Speech class object.
    #
	def __init__(self):
		## State-response pairs
		self.states = {}
		## Language of the responses
		self.language = ''

	def read_states(self):
		path = os.getcwd() + '/src/tum_alle_athome_speech/tum_alle_athome_speech_msgs/params/tts.csv'
		print (path)
		filename = rospy.get_param('filename', path)
		f = open(filename, 'r')
		# Read the language of the file
		language = f.readline()
		states = {}
		# Read the states and their responses
		content = f.readlines()
		print(content)
		content = [x.strip() for x in content]
		print(content)
		for line in content:
			text_state_pair = line.split(",")
			print(text_state_pair)
			states[text_state_pair[0]] = text_state_pair[1]
		self.language = language
		self.states = states
		print(self.states)

	def tts_with_state_callback(self, msg):
		if self.states == {}:
			self.read_states()
		# Connect to the text-to-speech action server
		client = SimpleActionClient('/tts', TtsAction)
		rospy.loginfo("SPEECH TTS NODE: Waiting for the server")
		if client.wait_for_server(timeout = rospy.Duration(2)):
			# Create a goal to say our sentence
			goal = TtsGoal()
			goal.rawtext.text = self.states[msg.state]
			goal.rawtext.lang_id = self.language
			# Send the goal and wait
			client.send_goal_and_wait(goal)

	def tts(self, req):
		# Connect to the text-to-speech action server
		client = SimpleActionClient('/tts', TtsAction)
		rospy.loginfo("SPEECH TTS NODE: Waiting for the server")
		if client.wait_for_server(timeout = rospy.Duration(2)):
			# Create a goal to say our sentence
			goal = TtsGoal()
			goal.rawtext.text = req.text
			goal.rawtext.lang_id = "en_GB"
			# Send the goal and wait
			client.send_goal_and_wait(goal)
			return True

	def tts_callback(self, msg):
		# Connect to the text-to-speech action server
		client = SimpleActionClient('/tts', TtsAction)
		rospy.loginfo("SPEECH TTS NODE: Waiting for the server")
		if client.wait_for_server(timeout = rospy.Duration(2)):
			# Create a goal to say our sentence
			goal = TtsGoal()
			goal.rawtext.text = msg.data
			goal.rawtext.lang_id = "en_GB"
			# Send the goal and wait
			client.send_goal_and_wait(goal)

	def tts_server(self):
	    s = rospy.Service('/tiago/speech/tts', srvTTS, self.tts)
	    rospy.loginfo("SPEECH TTS NODE: Ready to talk.")


if __name__ == '__main__':
	rospy.init_node('speech_node')
	tts = TextToSpeech()
	rospy.Subscriber('/tiago/speech/tts', String, tts.tts_callback)
	rospy.Subscriber('/tiago/speech/tts_with_state', msgTTSState, tts.tts_with_state_callback)
	tts.tts_server()
	rospy.spin()
