#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import sys

import rospy
from speech_msgs.msg import AudioData
from speech_msgs.msg import Speech
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

pub = rospy.Publisher('observation_ok',Speech,queue_size=100)
model = pickle.load(open('/home/yhirai/catkin_ws/src/deep_learning/scripts/model.pkl','rb'))

emotion = ['Angry','Disgust','Fear','Happy','Sadness','Surprise','Normal']

first = 0

write_wave = ""
wav_file = ""
audio = []
start = time.time()

def forward(x_data):
    x = Variable(x_data)
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
    global emotion
    global start        
    a_x = np.arange(1582)
    a_x = a_x.reshape((1,1582))
    mnist = {}
    for var in range(0,1582):
        a_x[0,var] = data.audio_buf[var]
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
    c_emotion = emotion[y.data[0].argmax()]
    print "Current_emotion : " + c_emotion
    print emotion
    #print y.data[0]
    print val_vec
    msg = Speech()
    msg.TrackingID = int(data.tracking_ID)
    msg.observation_value = [0] * 7
    for var in range(0,7):
        msg.observation_value[var] = val_vec[var]
    msg.emotion = c_emotion
    pub.publish(msg)
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

    rospy.Subscriber("convert_ok", AudioData, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()
