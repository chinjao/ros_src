#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import sys

import rospy
from speech_msgs.msg import AudioData
import subprocess
import codecs
import wave
import pyaudio
from numpy import *
import os, glob, random
import struct

first = 0

write_wave = ""
wav_file = ""
audio = []

def callback(data):
    
    global first
    global write_wave
    global wav_file
    global audio
    global pub
    if data.ok == True:
        print "qaaa"
    
        
def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("wav_ok", AudioData, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()
