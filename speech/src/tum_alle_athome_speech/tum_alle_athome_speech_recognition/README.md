# Speech recognition based on pocketsphinx

# Installation
## Dependencies Ubuntu 14.04
    
    sudo apt-get install autoconf bison swig gstreamer1.0-plugins-base python-gst-1.0 ros-indigo-audio-common libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev

## Dependencies Ubuntu 16.04

    sudo apt-get install autoconf bison swig gstreamer1.0-plugins-base python-gst-1.0 ros-kinetic-audio-common gstreamer1.0-pocketsphinx libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev

Optional (not required for the robocup challenge as we provide our own dictionary):

    sudo apt-get install pocketsphinx-hmm-en-hub4wsj pocketsphinx-lm-en-hub4

## Sphinxbase

    git clone https://github.com/cmusphinx/sphinxbase.git
    cd sphinxbase
    ./autogen.sh
    make
    make check
    make install

## Pocketsphinx

    cd ..
    git clone https://github.com/cmusphinx/pocketsphinx.git
    cd pocketsphinx
    ./autogen.sh
    make
    make check
    sudo make install

## Make sure that the gstreamer plugin has also been installed
**Ubuntu 14.04**: it should be located at: `/usr/local/lib/gstreamer-1.0/libgstpocketsphinx.so`
**Ubuntu 16.04**: it should be located at: `/usr/lib/x86_64-linux-gnu/gstreamer-1.0/libgstpocketsphinx.so`

# How to use
## Determine the correct microphone
Use the command:

    pacmd list-sources 

to list all connected microphones. Then chose the appropriate mic and note the string following `name:` right after `index: #`. That is the pulse audio microphone identifier.

## Set the microphone
Open the apropriate launch file and look for the string `mic_name`. The previously noted microphone's name should be entered there.

## Running
**Ubuntu 14.04**: To be able to recognise GStreamer you need to export the following var (maybe not needed on Ubuntu 16.04)

    export GST_PLUGIN_PATH=/usr/local/lib/gstreamer-1.0
    
Run the recogniser with:

    roslaunch tum_alle_athome_speech_recognition robocup_2017.launch

## See output, options
When running robocup_2017.launch:

    rostopic echo /tum_alle_athome_speech_recognition/output

To start / stop the recogniser, call:

    rosservice call /tum_alle_athome_speech_recognition/start
    rosservice call /tum_alle_athome_speech_recognition/stop


## Keyphrase recognition (old, ignore for now)
To recognise keyphrases only, add a file containing the keyphrases in the `language_models` folder. The file should contain one keyphrase per line. Then open the appropriate `.launch` file and add the file path of the keyphrase file as argument to `kws`.
**Attention:** When keyphrase recognition is running normal non-keyphrase speech won't be recognised.

# Helpful links
Build a new language model easily: http://www.speech.cs.cmu.edu/tools/lmtool-new.html  
https://cmusphinx.github.io/wiki/gstreamer/  
src: https://github.com/felixduvallet/pocketsphinx  
gstreamer1.0 fix: https://github.com/mikeferguson/pocketsphinx/pull/16/files  
https://stackoverflow.com/questions/35232989/how-to-use-pocketsphinx-5prealpha-with-gstreamer-1-0-in-python  
https://stackoverflow.com/questions/35467241/how-to-set-configuration-options-in-pocketsphinx-using-gstreamer  
https://github.com/skerit/cmusphinx/blob/master/pocketsphinx/src/gst-plugin/gstpocketsphinx.c  

