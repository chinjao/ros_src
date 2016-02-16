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
import time

import cPickle

import numpy as np
import six
import cv2
import os
import six.moves.cPickle as pickle

import chainer
from chainer import computational_graph as c
from chainer import cuda
import chainer.functions as F
from chainer import optimizers
from chainer import computational_graph as c
from chainer import Variable
from chainer.functions import *


first = 0

write_wave = ""
wav_file = ""
audio = []

pub = rospy.Publisher('convert_ok',AudioData,queue_size=100)

def callback(data):
    
    global first
    global write_wave
    global wav_file
    global audio
    global pub

    if data.ok == True:
  
        cmd = "ffmpeg -y -i /home/yhirai/openSMILE-2.1.0_other/input/bb.wav -ab 2 -vol 256 /home/yhirai/openSMILE-2.1.0_other/input/input.wav"
        subprocess.call(cmd,shell=True)
        
        cmd3 = "rm /home/yhirai/openSMILE-2.1.0_other/input/input.arff"
        subprocess.call(cmd3,shell=True)

        cmd2 = "/home/yhirai/openSMILE-2.1.0_other/bin/linux_x64_standalone_static/SMILExtract -C /home/yhirai/openSMILE-2.1.0_other/config/IS10_paraling.conf -I /home/yhirai/openSMILE-2.1.0_other/input/input.wav -O /home/yhirai/openSMILE-2.1.0_other/input/input.arff"
        subprocess.call(cmd2,shell=True)
        
        output = 0
        time.sleep(0.1)
        #f = open("/home/yhirai/openSMILE-2.1.0_other/input/new_input.arff",'w')
        for line2 in open("/home/yhirai/openSMILE-2.1.0_other/input/input.arff",'r'):
            line2 = line2.rstrip()
            if line2 == "@attribute":
                output = 0
            if output == 1:
                str_f = line2
                #f.write(line2)
            if line2 == "@data":
                output = 1
        out_l = range(1582)
        line2 = str_f.rstrip()
        List2 = line2.split(",")
        var2 = 1
        for var in range(0,1582):
            out_l[var] = float(List2[var2])
            var2 += 1
        msg = AudioData()
        msg.ok = True
        msg.tracking_ID = data.tracking_ID
        msg.audio_buf = out_l
        pub.publish(msg)

def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("wav_ok", AudioData, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()
