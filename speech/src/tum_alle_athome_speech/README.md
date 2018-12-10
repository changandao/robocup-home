# Dependencies

* pal_msgs: git@github.com:pal-robotics/pal_msgs.git
** pal_msgs dependency: git@github.com:ahornung/humanoid_msgs.git

If necessary: 

sudo apt-get install libatlas-base-dev


# Running
## Hotword recognition

    roslaunch tum_alle_athome_speech_hotword hotword_recogniser.py

Publishes commands to `/hotword_command`
Upon the hotwords: `say, move, follow, bring` calls the speech recognition

### Train new hotwords
Go to (https://snowboy.kitt.ai/dashboard)[https://snowboy.kitt.ai/dashboard], and log in. Then click create new hotword and record your voice. Finally, put it into `tum_alle_athome_speech_hotword/resources` and replace an existing hotword file.

## Speech recognition
    export GST_PLUGIN_PATH=/usr/local/lib/gstreamer-1.0
    roslaunch tum_alle_athome_speech_recognition robocup_2017.launch

Publishes the raw recognised text to `/tum_alle_athome_speech_recognition/output` and publishes categorised speech to: `/categorised_speech`

Speech defaults to paused, can be activated by saying a hotword or calling: `/tum_alle_athome_speech_recognition/speech_recog` with the arguments: 

    "action: ''
    command: ''" 

Where `action` is `start` or `stop` to stop the recognition and `command` is an empty string (used by the hotword node only)

The speech recognition node will call the reasoning node after the speech recognition node recognised a sentence. If the reasoning node is not started, the service call will fail. HOWEVER, the speech recognition itself will still work, **ONLY** the service call to the reasoning node failed!!


    
