#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from jsk_recognition_msgs.msg import HistogramWithRange, HistogramWithRangeBin
import numpy as np
from speech_msgs.msg import Bayesian

hirai_pub = rospy.Publisher("normal_array_hirai", Float32MultiArray)
uema_pub = rospy.Publisher("normal_array_uema", Float32MultiArray)
shimada_pub = rospy.Publisher("normal_array_shimada", Float32MultiArray)
yatsuduka_pub = rospy.Publisher("normal_array_yatsuduka", Float32MultiArray)
wakimoto_pub = rospy.Publisher("normal_array_wakimoto", Float32MultiArray)
teranishi_pub = rospy.Publisher("normal_array_teranishi", Float32MultiArray)
mukai_pub = rospy.Publisher("normal_array_mukai", Float32MultiArray)
yamamoto_pub = rospy.Publisher("normal_array_yamamoto", Float32MultiArray)
yano_pub = rospy.Publisher("normal_array_yano", Float32MultiArray)
fukuhara_pub = rospy.Publisher("normal_array_fukuhara", Float32MultiArray)
haruta_pub = rospy.Publisher("normal_array_haruta", Float32MultiArray)
haruna_pub = rospy.Publisher("normal_array_haruna", Float32MultiArray)
adachi_pub = rospy.Publisher("normal_array_adachi", Float32MultiArray)
ikegami_pub = rospy.Publisher("normal_array_ikegami", Float32MultiArray)
kawakita_pub = rospy.Publisher("normal_array_kawakita", Float32MultiArray)



def callback(data):
    print data.name

    print data.estimation_value
    a_x = [0] * 8
    a_x[0] = 1
    for var in range(0,7):
        a_x[var+1] = data.estimation_value[var]
    msg = Float32MultiArray()
    hist, bins = np.histogram(a_x,bins = 8)
    msg.data = data.estimation_value
    if data.name == "Hirai":
        hirai_pub.publish(msg)
    elif data.name == "Uema":
        uema_pub.publish(msg)
    elif data.name == "Yatsuduka":
        yatsuduka_pub.publish(msg)
    elif data.name == "Shimada":
        shimada_pub.publish(msg)
    elif data.name == "Teranishi":
        teranishi_pub.publish(msg)
    elif data.name == "Mukai":
        mukai_pub.publish(msg)
    elif data.name == "Wakimoto":
        wakimoto_pub.publish(msg)
    elif data.name == "Fukuhara":
        fukuhara_pub.publish(msg)
    elif data.name == "Yano":
        yano_pub.publish(msg)
    elif data.name == "Haruta":
        haruta_pub.publish(msg)
    elif data.name == "Haruna":
        haruna_pub.publish(msg)
    elif data.name == "Ikegami":
        ikegami_pub.publish(msg)
    elif data.name == "Adachi":
        adachi_pub.publish(msg)
    elif data.name == "Yamamoto":
        yamamoto_pub.publish(msg)
    elif data.name == "Kawakita":
        kawakita_pub.publish(msg)

def listener():
    rospy.init_node('create_hist_pub', anonymous = True)
    rospy.Subscriber("visualize", Bayesian, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
