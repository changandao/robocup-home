# Hotword detection node
This nodes detects a hotword, currently set to `tiago`. 

# Dependencies
Please install the following:

    sudo apt-get install sox python-pyaudio python3-pyaudio

# Running
Launch: 

    roslaunch tum_alle_athome_speech_hotword hotword_recogniser.launch

**How it works**: The hotword recogniser will listen to the speech input and when it recognises the hotword (currently `tiago`), it will call and start the speech recognition. Therefore, the speech recognition will have to be launched beforehand with

    roslaunch tum_alle_athome_speech_recognition robocup_2017.launch



