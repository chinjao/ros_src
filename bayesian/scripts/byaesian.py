# -*- coding: utf-8 -*-
import numpy as np
from numpy import matrix
import time
import rospy
from speech_msgs.msg import Emotion
from speech_msgs.msg import Bayesian

current_emotion_value = {}

name_timeline = {}

observation_value = [0]*7

estimation_v = [0.1] * 7

emotion_s = ['Angry','Disgust','Fear','Happy','Sadness','Surprise','Normal']

estimation_s = ['angry','disgust','fear','happy','sad','surprise','quiet']

Tr_matrix = {}
Tr_matrix["angry"] = {}
Tr_matrix["disgust"] = {}
Tr_matrix["fear"] = {}
Tr_matrix["happy"] = {}
Tr_matrix["sad"] = {}
Tr_matrix["surprise"] = {}
Tr_matrix["quiet"] = {}

emotion_trans["happy"]["happy"] = 0.421;
emotion_trans["happy"]["quiet"] = 0.362;
emotion_trans["happy"]["sad"] = 0.061;
emotion_trans["happy"]["surprise"] = 0.06;
emotion_trans["happy"]["angry"] = 0.027;
emotion_trans["happy"]["fear"] = 0.034;
emotion_trans["happy"]["disgust"] = 0.032;

emotion_trans["quiet"]["happy"] = 0.213;
emotion_trans["quiet"]["quiet"] = 0.509;
emotion_trans["quiet"]["sad"] = 0.09;
emotion_trans["quiet"]["surprise"] = 0.055;
emotion_trans["quiet"]["angry"] = 0.039;
emotion_trans["quiet"]["fear"] = 0.051;
emotion_trans["quiet"]["disgust"] = 0.042;

emotion_trans["sad"]["happy"] = 0.084;
emotion_trans["sad"]["quiet"] = 0.296;
emotion_trans["sad"]["sad"] = 0.32;
emotion_trans["sad"]["surprise"] = 0.058;
emotion_trans["sad"]["angry"] = 0.108;
emotion_trans["sad"]["fear"] = 0.064;
emotion_trans["sad"]["disgust"] = 0.068;

emotion_trans["surprise"]["happy"] = 0.19;
emotion_trans["surprise"]["quiet"] = 0.264;
emotion_trans["surprise"]["sad"] = 0.091;
emotion_trans["surprise"]["surprise"] = 0.243;
emotion_trans["surprise"]["angry"] = 0.086;
emotion_trans["surprise"]["fear"] = 0.076;
emotion_trans["surprise"]["disgust"] = 0.048;

emotion_trans["angry"]["happy"] = 0.056;
emotion_trans["angry"]["quiet"] = 0.262;
emotion_trans["angry"]["sad"] = 0.123;
emotion_trans["angry"]["surprise"] = 0.075;
emotion_trans["angry"]["angry"] = 0.293;
emotion_trans["angry"]["fear"] = 0.069;
emotion_trans["angry"]["disgust"] = 0.121;

emotion_trans["fear"]["happy"] = 0.05;
emotion_trans["fear"]["quiet"] = 0.244;
emotion_trans["fear"]["sad"] = 0.137;
emotion_trans["fear"]["surprise"] = 0.101;
emotion_trans["fear"]["angry"] = 0.096;
emotion_trans["fear"]["fear"] = 0.279;
emotion_trans["fear"]["disgust"] = 0.092;

emotion_trans["disgust"]["happy"] = 0.047;
emotion_trans["disgust"]["quiet"] = 0.252;
emotion_trans["disgust"]["sad"] = 0.092;
emotion_trans["disgust"]["surprise"] = 0.056;
emotion_trans["disgust"]["angry"] = 0.164;
emotion_trans["disgust"]["fear"] = 0.075;
emotion_trans["disgust"]["disgust"] = 0.313;


def initialize_emo(name):
    global current_emotion_value
    current_emotion_value[name] = {}
    current_emotion_value[name]["happy"] = 0.201
    current_emotion_value[name]["quiet"] = 0.378
    current_emotion_value[name]["sad"] = 0.118
    current_emotion_value[name]["surprise"] = 0.076
    current_emotion_value[name]["angry"] = 0.083
    current_emotion_value[name]["fear"] = 0.070
    current_emotion_value[name]["disgust"] = 0.073
    name_timeline[name] = time.time()

def callback(data):
    global observation_value
    global current_emotion_value
    name = data.name
    for var in range(0,7):
        observation_value[var] = data->observation_value[var]
    
    if name in current_emotion_value == True:
        initialize_emo(name)
    
    

def listener():
    rospy.init_node('bayesian_deep', anonymous = True)
    rospy.Subscriber("emo_and_name", Emotion, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
