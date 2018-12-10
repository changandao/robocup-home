#!/usr/bin/env python

"""
recogniser.py is a wrapper for pocketsphinx.
  parameters:
    ~lm - filename of language model
    ~dict - filename of dictionary
    ~mic_name - set the pulsesrc device name for the microphone input.
           To list audio device info on your machine, in a terminal type:
           pacmd list-sources
    ~hmm - directory name of acoustic model.
            By default,
            it would be "/usr/share/pocketsphinx/model/hmm/en_US/hub4wsj_sc_8k"
            You can download custom acoustic models from CMU Sphix web site:
            https://sourceforge.net/projects/cmusphinx/files/Acoustic%20and%20Language%20Models/
    ~kws - kws which will always be detected
  publications:
    ~output (std_msgs/String) - text output
    tum_alle_athome_speech_recognition/categorised_speech
    (tum_alle_athome_common_msgs/msgCategorisedSPeech):
    speech partitioned into three categories
  services:
    srvSpeechRecognition - stops/starts the speech recognition service
    and modifies robot command, called by this package
"""

import roslib; roslib.load_manifest('tum_alle_athome_speech_recognition')
import rospy
import os, os.path
import commands

from gi import pygtkcompat
import gi

gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst
GObject.threads_init()
Gst.init(None)

gst = Gst

pygtkcompat.enable()
pygtkcompat.enable_gtk(version='3.0')
import gtk

from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse
from audio_common_msgs.msg import AudioData

# For parsing files to split sentences accordingly
import xml.etree.ElementTree

from tum_alle_common_msgs.msg import msgCategorisedSpeech
from tum_alle_common_msgs.srv import srvPerformTask
from tum_alle_common_msgs.srv import srvSpeechRecognition, srvSpeechRecognitionResponse

# Debug stuff
os.environ["GST_DEBUG_DUMP_DOT_DIR"] = "/tmp"
os.putenv('GST_DEBUG_DUMP_DIR_DIR', '/tmp')


current_dir = os.path.abspath(os.path.dirname(__file__))

class recogniser(object):

    """ GStreamer based speech recogniser. """

    def __init__(self):
        self.started = False
        rospy.init_node("recogniser")

        self._mic_name_param = "~mic_name"
        self._lm_param = "~lm"
        self._dic_param = "~dict"
        self._audio_topic_param = "~audio_msg_topic"
        self._hmm_param = "~hmm"
        self._kws_param = "~kws"
        self._semantics_perform_task_srv_param = "~perform_task_srv"

        self._locations_file = os.path.join(current_dir,
                                    "../language_models/Locations.xml")
        self._names_file = os.path.join(current_dir,
                                    "../language_models/Names.xml")
        self._objects_file = os.path.join(current_dir,
                                    "../language_models/Objects.xml")

        self._locs = xml.etree.ElementTree.parse(self._locations_file).getroot()
        self._names = xml.etree.ElementTree.parse(self._names_file).getroot()
        self._objects = xml.etree.ElementTree.parse(self._objects_file).getroot()

        self.perform_task_msg_command = ""
        self.perform_task_msg_destination = ""
        self.perform_task_msg_obj = ""
        self.perform_task_msg_location = ""
        self.perform_task_msg_confirmation = ""

        self.init_gstreamer()
        self.init_ros()
        self.init_pocketsphinx()

        # Audio ROS topic, if this is set the recogniser will subscribe to
        # AudioData messages on this topic.
        self._ros_audio_topic = None
        self._app_source = None  # The gstreamer appsrc element

        if rospy.has_param(self._lm_param) and rospy.has_param(self._dic_param):
            rospy.loginfo("All params set, ready to start")
        else:
            rospy.logwarn("lm and dic parameters need to be "
                          "set to start recogniser.")

        # Stop recogniser again, to be launched by hotword detection initially
        #self.stop_recogniser()

    def init_gstreamer(self):
        # Configure mics with gstreamer launch config
        if rospy.has_param(self._mic_name_param):
            self.device_name = rospy.get_param(self._mic_name_param)
            self.device_index = self.pulse_index_from_name(self.device_name)
            self.launch_config = "pulsesrc device=" + str(self.device_index)
            rospy.loginfo(
                            "Using: pulsesrc device=%s name=%s",
                            self.device_index,
                            self.device_name
                        )
        elif rospy.has_param('~source'):
            # common sources: 'alsasrc'
            self.launch_config = rospy.get_param('~source')
        elif rospy.has_param(self._audio_topic_param):
            # Use ROS audio messages as input: Use an appsrc to pass AudioData
            # messages to the gstreamer pipeline. Use 'mad' plugin to decode
            # mp3-formatted messages.
            self.launch_config = 'appsrc name=appsrc ! mad'
            self._ros_audio_topic = rospy.get_param(self._audio_topic_param)
            rospy.loginfo('Using ROS audio messages as input. Topic: {}'.
                          format(self._ros_audio_topic))
        else:
            self.launch_config = ''

        self.launch_config += ' ! audioconvert ! audioresample ' \
                            + '! pocketsphinx name=asr ! fakesink'

        # Initialise gstreamer pipeline
        self.pipeline = gst.parse_launch(self.launch_config)
        if not self.pipeline:
            rospy.logerr('Could not create gstreamer pipeline.')
            #return
        rospy.loginfo('gstreamer pipeline created.')
        #rospy.loginfo("Audio input: {}".format(self.launch_config))

    def init_ros(self):
        # Configure ROS settings
        rospy.on_shutdown(self.shutdown)
        self.pub = rospy.Publisher('~output', String, queue_size=10)
        self.categorised_pub = rospy.Publisher(
                            'categorised_speech',
                            msgCategorisedSpeech,
                            queue_size=10
                            )
        __ = rospy.Service(
                        '/tum_alle_athome_speech_recognition/speech_recog',
                        srvSpeechRecognition,
                        self.speech_recognition_srv_cb
                        )

        if rospy.has_param(self._semantics_perform_task_srv_param):
            self.semantics_perform_task_srv = rospy.get_param(
                                    self._semantics_perform_task_srv_param)
        else:
            rospy.logerr('No semantics service configured in .launch')

    def init_pocketsphinx(self):
        self.asr = self.pipeline.get_by_name('asr')
        # Configure language model
        if rospy.has_param(self._lm_param):
            lm = rospy.get_param(self._lm_param)
            if not os.path.isfile(lm):
                rospy.logerr(
                    'Language model file does not exist: {}'.format(lm))
                return
        else:
            rospy.logerr('Recogniser not started. Please specify a '
                         'language model file.')
            return

        if rospy.has_param(self._dic_param):
            dic = rospy.get_param(self._dic_param)
            if not os.path.isfile(dic):
                rospy.logerr(
                    'Dictionary file does not exist: {}'.format(dic))
                return
        else:
            rospy.logerr('Recogniser not started. Please specify a dictionary.')
            return

        if rospy.has_param(self._hmm_param):
            hmm = rospy.get_param(self._hmm_param)
        if rospy.has_param(self._kws_param):
            kws = rospy.get_param(self._kws_param)

        self.asr.set_property('lm', lm)
        self.asr.set_property('dict', dic)
        if rospy.has_param(self._hmm_param):
            self.asr.set_property('hmm', hmm)
        if rospy.has_param(self._kws_param):
            self.asr.set_property('kws', kws)

        # Get bus and set callback function called when received msg
        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus_id = self.bus.connect('message::element', self.element_message)

    def start_recogniser(self):
        if self.pipeline.get_state(gst.CLOCK_TIME_NONE)[1] == gst.State.PLAYING:
        #if self.started is True:
            rospy.loginfo("Recogniser already running... abort starting again")
            return
        #rospy.loginfo('Starting recogniser... pipeline: {}'.format(
        #    self.launch_config))

        # If the ros audio topic exists, we subscribe to AudioData messages.
        # Also make sure the appsource element was created properly.
        if self._ros_audio_topic:
            self._app_source = self.pipeline.get_by_name('appsrc')
            rospy.loginfo('Subscribing to AudioData on topic: {}'.format(
                self._ros_audio_topic))
            rospy.Subscriber(
                            self._ros_audio_topic,
                            AudioData,
                            self.on_audio_message)
            if not self._app_source:
                rospy.logerr('Error getting the appsrc element.')
                return

        self.pipeline.set_state(gst.State.PLAYING)
        rospy.loginfo("Starting recogniser...")
        self.pipeline.get_state(gst.CLOCK_TIME_NONE)
        #self.started = True
        rospy.loginfo('Recogniser started!')

    def pulse_index_from_name(self, name):
        output = commands.getstatusoutput(
            ("pacmd list-sources | grep -B 1 'name: <" + name +
             ">' | grep -o -P '(?<=index: )[0-9]*'"))

        if len(output) == 2:
            return output[1]
        else:
            raise Exception("Error. pulse index doesn't exist for name: {}".
                            format(name))

    def start(self):
        self.start_recogniser()
        return EmptyResponse()

    def stop(self):
        self.stop_recogniser()
        return EmptyResponse()

    def element_message(self, bus, msg):
        """ Receive application messages from the bus. """
        msgtype = msg.get_structure().get_name()
        if msgtype != 'pocketsphinx':
            rospy.loginfo("Received message with wrong type: " + msgtype)
            return

        if msg.get_structure().get_value('final'):
            self.final_result(msg.get_structure().get_value('hypothesis'),
                              msg.get_structure().get_value('confidence'))

        elif msg.get_structure().get_value('hypothesis'):
            self.partial_result(msg.get_structure().get_value('hypothesis'))

    def partial_result(self, hyp):
        """ Delete any previous selection, insert text and select it. """
        rospy.logdebug("Partial: " + hyp)

    def final_result(self, hyp, confidence):
        """ Insert the final result. """
        msg = String()
        msg.data = str(hyp.lower())
        rospy.loginfo(
                        'Detected string: %s',
                        msg.data
                        )
        # Stop recogniser until started again by hotword/reasoning
        self.stop()
        self.pub.publish(msg)
        self.split_text_into_logic_parts(msg.data)

    def on_audio_message(self, audio):
        # Callback for ROS audio messages -- emits the audio data to the
        # gstreamer pipeline through the appsrc.
        rospy.logdebug('Received audio packet of length {}'.format(
            len(audio.data)))
        if self._app_source:
            self._app_source.emit('push-buffer',
                                  gst.Buffer(str(bytearray(audio.data))))

    def split_text_into_logic_parts(self, message):
        """
        Splits the recognised string and categorises it into:
        location, name or object
        """
        msg = msgCategorisedSpeech()

        for word in message.split():
            for child in self._locs:
                if child.get('name') == word:
                    self.perform_task_msg_location = str(word)
                    msg.location = str(word)
                    break

            for child in self._names:
                if child.text.lower() == word:
                    self.perform_task_msg_destination = str(word)
                    msg.name = str(word)
                    break

            for child in self._objects:
                if child.get('name') == word:
                    self.perform_task_msg_obj = str(word)
                    msg.object = str(word)
                    break

            # ja/nein because yes/no was interpreted as true/false
            if "yes" == word:
                self.perform_task_msg_confirmation = "ja"

            if "no" == word:
                self.perform_task_msg_confirmation = "nein"

        self.categorised_pub.publish(msg)
        self.perform_task_client()

    def speech_recognition_srv_cb(self, srv):
        rospy.loginfo("Speech_srv callback received")
        if 'start' == srv.action:
            self.start()
        elif 'stop' == srv.action:
            self.stop()
        self.perform_task_msg_command = srv.command
        # Has return a list, idk why
        res = srvSpeechRecognitionResponse(True)
        res.response = True
        return res

    def perform_task_client(self):
        # rospy.wait_for_service(self.semantics_bring_object_srv)
        rospy.loginfo("Call perf task with: \n:" +
                        "command: " + self.perform_task_msg_command +
                        "dest: " + self.perform_task_msg_destination +
                        "obj: " + self.perform_task_msg_obj +
                        "loc: " + self.perform_task_msg_location +
                        "confirm: " + self.perform_task_msg_confirmation
                        )
        try:
            rospy.loginfo("Call perf task with: \n:" +
                        "command: " + self.perform_task_msg_command +
                        "dest: " + self.perform_task_msg_destination +
                        "obj: " + self.perform_task_msg_obj +
                        "loc: " + self.perform_task_msg_location +
                        "confirm: " + self.perform_task_msg_confirmation
                        )
            perform_task = rospy.ServiceProxy(
                            self.semantics_perform_task_srv,
                            srvPerformTask
                            )
            __ = perform_task(
                    self.perform_task_msg_command,
                    self.perform_task_msg_destination,
                    self.perform_task_msg_obj,
                    self.perform_task_msg_location,
                    "",
                    "",
                    self.perform_task_msg_confirmation
                    )
        except rospy.ServiceException, e:
            print("The service call to reasoning failed, speech still works: %s" % e)
        # Reset confirmation string to default to empty
        self.perform_task_msg_confirmation = ""
        rospy.loginfo("Perform task called")
        # self.on_debug_activate()

    def stop_recogniser(self):
        rospy.loginfo("Trying to stop recogniser...")
        if self.pipeline.get_state(gst.CLOCK_TIME_NONE)[1] == gst.State.PAUSED:
            rospy.loginfo("Recogniser already stopped, aborting stopping again")
            return
        self.pipeline.set_state(gst.State.PAUSED)
        self.pipeline.get_state(gst.CLOCK_TIME_NONE)
        rospy.loginfo("Recogniser stopped!")

    def on_debug_activate(self):
        """ Used to output a graph of the gstreamer pipeline """
        dotfile = "/tmp/supersimple-debug-graph.dot"
        pngfile = "/tmp/supersimple-pipeline.png"
        if os.access(dotfile, os.F_OK):
            os.remove(dotfile)
        if os.access(pngfile, os.F_OK):
            os.remove(pngfile)
        gst.debug_bin_to_dot_file (self.pipeline, \
        gst.DebugGraphDetails.ALL, 'supersimple-debug-graph')

    def shutdown(self):
        """ Delete any remaining parameters so they don't affect next launch """
        for param in [self._mic_name_param, self._lm_param, self._dic_param,
                      self._audio_topic_param, self._hmm_param, self._kws_param]:
            if rospy.has_param(param):
                rospy.delete_param(param)

        # Stop & disconnect pipeline
        self.pipeline.remove(self.asr)
        self.bus.disconnect(self.bus_id)

        self.pipeline.set_state(gst.State.NULL)
        self.pipeline.get_state(gst.CLOCK_TIME_NONE)
        """ Shutdown the GTK thread. """
        gtk.main_quit()


if __name__ == "__main__":
    start = recogniser()
    gtk.main()
