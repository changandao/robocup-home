#!/usr/bin/env python
import rospy
import os, os.path, sys, time
import time

import nltk
import aiml

from std_msgs.msg import String
from tum_alle_athome_speech_msgs.srv import srvQuestion

absbasepath = os.path.abspath(os.path.dirname(__file__))

## Class storing basic tokenization methods
#
class Speech_AIML:

    ## Create a ProcessText class object.
    #
    def __init__(self):
        ## AIML kernel
        self.kernel = aiml.Kernel()
        self.kernel.learn(os.path.join(absbasepath,"../training/AIML_bots/competition.aiml"))
        self.kernel.learn(os.path.join(absbasepath,"../training/AIML_bots/time.aiml"))
        self.kernel.learn(os.path.join(absbasepath,"../training/AIML_bots/general.aiml"))
        ## Sentence processed
        self.last_question = ''
        self.last_answer = ''

    def get_answer(self, text):
        # Connect to the text-to-speech action server
        response = self.kernel.respond(text)
        if response == "":
            rospy.loginfo("SPEECH AIML NODE: Unknown question.")
            return response
        else:
            final = response.replace("$month", time.strftime("%B"))
            final = final.replace("$time", time.strftime("%H:%M"))
            final = final.replace("$day", time.strftime("%A, %m of %B"))
            final = final.replace("$date", time.strftime("%e of %B, %Y"))
            final = final.replace("$last", self.last_question)
            final = final.replace("$year", time.strftime("%Y"))
            self.last_question = text
            self.last_answer = final
            print (final)
            return final

    def question_callback(self, msg):
        response = self.get_answer(msg.data)


    def question(self, req):
        response = self.get_answer(req.question)
        return response

    def question_server(self):
        s = rospy.Service('/tiago/speech/predefined_answer', srvQuestion, self.question)

if __name__ == '__main__':
    rospy.init_node('speech_aiml')
    rospy.loginfo("SPEECH AIML: Initializing")
    aiml = Speech_AIML()
    rospy.Subscriber('/tiago/speech/question', String, aiml.question_callback)
    aiml.question_server()
    rospy.spin()
