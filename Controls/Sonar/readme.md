# Sonar
This is a living documentation, it is not finished yet.

## ROS Audio Configuration

ROS (via Ubuntu) can use externally-connected audio input devices to stream audio data on a special rostopic. This is a brief guide to ROS Audio setup and some explanations of things that you may encounter along the way. This guide assumes you already have ROS setup on an Ubuntu installation, and that you are familiar with the basic tools of ROS, like nodes, topics, publishers and subscribers.

First, install ROS Audio by inputting the following command into Terminal. This installation of ROS Audio was done on Ubuntu 18.04 using ROS Melodic. If your ROS version is different, replace 'melodic' with the name of your installation (i.e. `ros-noetic-audio-common`)
```
sudo apt-get install ros-melodic-audio-common
```
Next, initialise ROS by running `roscore` in a new Terminal window. Then, run the following command in a different Terminal window:
```
roslaunch audio_capture capture.launch
```
Running this will display some messages, some of which may resemble the following:
```
SUMMARY
========

PARAMETERS
 * /audio/audio_capture/bitrate: 128
 * /audio/audio_capture/channels: 1
 * /audio/audio_capture/device: 
 * /audio/audio_capture/format: mp3
 * /audio/audio_capture/sample_format: S16LE
 * /audio/audio_capture/sample_rate: 16000
 * /rosdistro: melodic
 * /rosversion: 1.14.10

NODES
  /audio/
    audio_capture (audio_capture/audio_capture)
```
What you have just done is launch the ROS Audio Capture node. This node directly takes audio signals from your input device (i.e. microphone, hydrophone) and streams it in mp3 format. The default bitrate and sample rate are as shown above.

-WIP-
