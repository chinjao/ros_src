#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy
from speech_msgs.msg import Emotion
from speech_msgs.msg import Bayesian

import threading

current_emotion_value = {}

name_timeline = {}

persons = 0

persons_name = []

pub = rospy.Publisher('bayes_result',Bayesian,queue_size=100)

pub_vis = rospy.Publisher('visualize',Bayesian,queue_size=100)


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

Tr_matrix["happy"]["happy"] = 0.421
Tr_matrix["happy"]["quiet"] = 0.362
Tr_matrix["happy"]["sad"] = 0.061
Tr_matrix["happy"]["surprise"] = 0.06
Tr_matrix["happy"]["angry"] = 0.027
Tr_matrix["happy"]["fear"] = 0.034
Tr_matrix["happy"]["disgust"] = 0.032

Tr_matrix["quiet"]["happy"] = 0.213
Tr_matrix["quiet"]["quiet"] = 0.509
Tr_matrix["quiet"]["sad"] = 0.09
Tr_matrix["quiet"]["surprise"] = 0.055
Tr_matrix["quiet"]["angry"] = 0.039
Tr_matrix["quiet"]["fear"] = 0.051
Tr_matrix["quiet"]["disgust"] = 0.042

Tr_matrix["sad"]["happy"] = 0.084
Tr_matrix["sad"]["quiet"] = 0.296
Tr_matrix["sad"]["sad"] = 0.32
Tr_matrix["sad"]["surprise"] = 0.058
Tr_matrix["sad"]["angry"] = 0.108
Tr_matrix["sad"]["fear"] = 0.064
Tr_matrix["sad"]["disgust"] = 0.068

Tr_matrix["surprise"]["happy"] = 0.19
Tr_matrix["surprise"]["quiet"] = 0.264
Tr_matrix["surprise"]["sad"] = 0.091
Tr_matrix["surprise"]["surprise"] = 0.243
Tr_matrix["surprise"]["angry"] = 0.086
Tr_matrix["surprise"]["fear"] = 0.076
Tr_matrix["surprise"]["disgust"] = 0.048

Tr_matrix["angry"]["happy"] = 0.056
Tr_matrix["angry"]["quiet"] = 0.262
Tr_matrix["angry"]["sad"] = 0.123
Tr_matrix["angry"]["surprise"] = 0.075
Tr_matrix["angry"]["angry"] = 0.293
Tr_matrix["angry"]["fear"] = 0.069
Tr_matrix["angry"]["disgust"] = 0.121

Tr_matrix["fear"]["happy"] = 0.05
Tr_matrix["fear"]["quiet"] = 0.244
Tr_matrix["fear"]["sad"] = 0.137
Tr_matrix["fear"]["surprise"] = 0.101
Tr_matrix["fear"]["angry"] = 0.096
Tr_matrix["fear"]["fear"] = 0.279
Tr_matrix["fear"]["disgust"] = 0.092

Tr_matrix["disgust"]["happy"] = 0.047
Tr_matrix["disgust"]["quiet"] = 0.252
Tr_matrix["disgust"]["sad"] = 0.092
Tr_matrix["disgust"]["surprise"] = 0.056
Tr_matrix["disgust"]["angry"] = 0.164
Tr_matrix["disgust"]["fear"] = 0.075
Tr_matrix["disgust"]["disgust"] = 0.313;

class Timeline(threading.Thread):
    def getVarsNames( _vars, symboltable ) :
        """
        This is wrapper of getVarName() for a list references.
        """
        return [ getVarName( var, symboltable ) for var in _vars ]

    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        global persons
        global persons_name
        global current_emotion_value
        global name_timeline
        global Tr_matrix
        global pub_vis
                  
        value = [0] * 7
        estimation_s = ['angry','disgust','fear','happy','sad','surprise','quiet']
        msg_time = Bayesian()

        while True:
            time.sleep(0.2)
            for num in range(0,persons):
                current_time =  time.time() - name_timeline[persons_name[num]]
                if current_time > 3:
                    for var in range(0,7):
                        for var2 in range(0,7):
                            value[var] += Tr_matrix[estimation_s[var2]][estimation_s[var]]  * current_emotion_value[persons_name[num]][estimation_s[var2]]
                    all = 0
                    for var in range(0,7):
                        all += value[var]
    
                    for var in range(0,7):
                        current_emotion_value[persons_name[num]][estimation_s[var]] = value[var] / all
                    print current_emotion_value[persons_name[num]]
                    name_timeline[persons_name[num]] = time.time()
                    msg_time.estimation_value = [0] * 7
                    #msg_time.emotion = now_emo
                    msg_time.name = persons_name[num]
                    for var in range(0,7):
                        msg_time.estimation_value[var] = current_emotion_value[persons_name[num]][estimation_s[var]]
                    print "update"
                    pub_vis.publish(msg_time)

                    
                    


def initialize_emo(name):
    global current_emotion_value
    global persons_name
    global persons
    print name
    current_emotion_value[name] = {}
    current_emotion_value[name]["happy"] = 0.201
    current_emotion_value[name]["quiet"] = 0.378
    current_emotion_value[name]["sad"] = 0.118
    current_emotion_value[name]["surprise"] = 0.076
    current_emotion_value[name]["angry"] = 0.083
    current_emotion_value[name]["fear"] = 0.070
    current_emotion_value[name]["disgust"] = 0.073
    name_timeline[name] = time.time()
    persons_name.append(name)
    persons += 1

def pub_info(name,now_emo,estimation_s):
    global current_emotion_value
    global pub
    global pub_vis


    msg = Bayesian()
    msg.estimation_value = [0] * 7
    msg.emotion = now_emo
    msg.name = name
    for var in range(0,7):
        msg.estimation_value[var] = current_emotion_value[name][estimation_s[var]]
    time.sleep(0.4)

    pub.publish(msg)
    pub_vis.publish(msg)

def multiply(name,eee_value):
    global current_emotion_value
    global Tr_matrix
    estimation_v = [0] * 7
    value = [0] * 7
    estimation_s = ['angry','disgust','fear','happy','sad','surprise','quiet']

    for var in range(0,7):
        for var2 in range(0,7):
            value[var] += eee_value[var] * Tr_matrix[estimation_s[var2]][estimation_s[var]]  * current_emotion_value[name][estimation_s[var2]]

    all = 0
    for var in range(0,7):
        all += value[var]
    
    for var in range(0,7):
        current_emotion_value[name][estimation_s[var]] = value[var] / all
        estimation_v[var] = value[var] / all
    
    print current_emotion_value[name]
    
    now_emo = estimation_s[max(xrange(len(estimation_v)),key = lambda i:estimation_v[i])]

    print "Current_emotion : " + now_emo
    
    name_timeline[name] = time.time()
    pub_info(name,now_emo,estimation_s)
    
def callback(data):
 
    global current_emotion_value

    observation_value = [0.1] * 7
    name = data.name
    for var in range(0,7):
        observation_value[var] = data.observation_value[var]

    if current_emotion_value.has_key(name) == False:
        initialize_emo(name)

    
    multiply(name,observation_value)



def listener():
    rospy.init_node('bayesian_deep', anonymous = True)
    rospy.Subscriber("emo_and_name", Emotion, callback)
    rospy.spin()

if __name__ == '__main__':
    
    th = Timeline()
    th.setDaemon(True)
    th.start()
    
    listener()
