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


model = pickle.load(open('model.pkl','rb'))

emotion = ['Angry','Disgust','Fear','Happy','Sadness','Surprise','Normal']

first = 0

write_wave = ""
wav_file = ""
audio = []
start = time.time()

def forward(x_data):
    x = Variable(x_data)
    '''
    h = F.max_pooling_2d(F.relu(model.l1(x)), ksize=5,stribe=2,pad=2)
    
    h = F.max_pooling_2d(F.relu(model.l2(h)))
    '''
    h1 = F.sigmoid(model.l1(x))
    h2 = F.sigmoid(model.l2(h1))
    
    y = model.l3(h2)
    y2 = softmax(model.l3(h2))
   
    return y2


def callback(data):
    
    global first
    global write_wave
    global wav_file
    global audio
    global pub
    global emotion
    global start
    if data.ok == True:
        start = time.time()
        #time.sleep(1)
        cmd = "ffmpeg -y -i ../../../../openSMILE-2.1.0_other/input/bb.wav -ab 2 ../../../../openSMILE-2.1.0_other/input/input.wav"
        subprocess.call(cmd,shell=True)
        
        cmd3 = "rm /home/yhirai/openSMILE-2.1.0_other/input/input.arff"
        subprocess.call(cmd3,shell=True)

        cmd2 = "/home/yhirai/openSMILE-2.1.0_other/bin/linux_x64_standalone_static/SMILExtract -C /home/yhirai/openSMILE-2.1.0_other/config/IS10_paraling.conf -I /home/yhirai/openSMILE-2.1.0_other/input/input.wav -O /home/yhirai/openSMILE-2.1.0_other/input/input.arff"
        subprocess.call(cmd2,shell=True)

        output = 0
        f = open("/home/yhirai/openSMILE-2.1.0_other/input/new_input.arff",'w')
        for line2 in open("/home/yhirai/openSMILE-2.1.0_other/input/input.arff",'r'):
            line2 = line2.rstrip()
            if line2 == "@attribute":
                output = 0
            if output == 1:
                f.write(line2)
            if line2 == "@data":
                output = 1
        out_l = range(1582)
        
        f.close()
        for line in open("/home/yhirai/openSMILE-2.1.0_other/input/new_input.arff"):
            line = line.rstrip()
            List2 = line2.split(",")
            var2 = 1
            for var in range(0,1582):
                out_l[var] = List2[var2]
                var2 += 1
            f = open("/home/yhirai/openSMILE-2.1.0_other/input/x.txt",'w')
            for var in range(0,1582):
                out_l[var] += "\n"
                f.write(out_l[var])
        print type(out_l)
        
        a_x = np.arange(1582)
        a_x = a_x.reshape((1,1582))
        mnist = {}
        for var in range(0,1582):
            a_x[0,var] = float(out_l[var])
        #x_test = a_x.astype(np.float32)
        
        mnist['data'] = a_x
        mnist['data'] = mnist['data'].astype(np.float32)
        mnist['data'] /= 255
        x_test = mnist['data']
        
        y = forward(x_test)
        val_vec = [0] * 7
        all = y.data[0][0] + y.data[0][1] + y.data[0][2] + y.data[0][3] +y.data[0][4] + y.data[0][5] + y.data[0][6]
        
        val_vec[0] = y.data[0][0] / all
        val_vec[1] = y.data[0][1] / all
        val_vec[2] = y.data[0][2] / all
        val_vec[3] = y.data[0][3] / all
        val_vec[4] = y.data[0][4] / all
        val_vec[5] = y.data[0][5] / all
        val_vec[6] = y.data[0][6] / all
        print "Current_emotion : " + emotion[y.data[0].argmax()]
        print emotion
        #print y.data[0]
        print val_vec
        #end = time.time() - start
        #print end
        #print all
        #print all
        
        
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
