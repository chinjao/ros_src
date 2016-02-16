#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
u"""Docomoの雑談対話APIを使ってチャットできるスクリプト
"""
import rospy
from std_msgs.msg import String

import sys
import urllib2
import json
import subprocess
import wave
import os
import random
 
APP_URL = 'https://api.apigw.smt.docomo.ne.jp/voiceText/v1/textToSpeech'

class DocomoChat(object):
    u"""Docomoの雑談対話APIでチャット"""
 
    def __init__(self, api_key):
        super(DocomoChat, self).__init__()
        self.api_url = APP_URL + '?APIKEY=%s'%(api_key)
        self.context, self.mode = None, None
 
    def __send_message(self, input_message='', steat = '', custom_dict=None):
        
        text = "text="+input_message+steat
        print text
        request = urllib2.Request(self.api_url, text)
        request.add_header('Content-Type', 'application/x-www-form-urlencoded')
        try:
            response = urllib2.urlopen(request)
        except Exception as e:
            print e
            sys.exit()
        return response
 
    
    def send_and_get(self, input_message,steat):
        response = self.__send_message(input_message,steat)
        #received_message = self.__process_response(response)
        return response
 
 
def main():
    api_key = '6b4541653859576b634843343566506f71662f57475732544b504e6b526f4e61616657667a4677637a6344'
    chat = DocomoChat(api_key)
    rospy.init_node('listener', anonymous=True)
    message = ""
    steat = ""

    while True:
        res = rospy.wait_for_message("chatter",String)
        a = int(res.data)
        print a
        #global a 
	if a == 0:
            print a
            #message = "sadness"
            message_choice = ['泣かないで下さい','どうしました？','元気だして！','なぜ悲しそうなの？']
            message = random.choice(message_choice)
            steat = "&speaker=hikari&emotion=sadness&emotion_level=2&pitch=80&speed=80"
        elif a == 1:
            print a
            #message = "neutral"
            message_choice = ['頑張ろう','良いことありますように！','ふわぁー']
            message =  random.choice(message_choice)
            steat = "&speaker=hikari"
        elif a == 2 or a == 3 or a == 6:
            print a
            #message = "contempt&disgust&fear"
            message_choice = ['なになに？','どうかしました？','怖い怖い！']
            message =  random.choice(message_choice)
            steat = "&speaker=hikari&speed=110"
	elif a == 4:
            print a
            #message = "anger"
            message_choice = ['何かあいました？','機嫌直して下さい','怖いです']
            message =  random.choice(message_choice)
            steat = "&speaker=hikari&emotion=sadness&emotion_level=2&pitch=80&speed=80"
	elif a == 5:
            print a
            #message = "surprise"
            message_choice = ['びっくりした！','驚かせないで','きゃー！']
            message =  random.choice(message_choice)
            steat = "&speaker=hikari&emotion=happiness&emotion_level=2&pitch=130&speed=110"
	elif a == 7:
            print a
            #message = "happiness"
            message_choice = ['嬉しそう','いいことありました？','幸せそう']
            message =  random.choice(message_choice)
            steat = "&speaker=hikari&emotion=happiness&emotion_level=2&pitch=120&speed=105"
        else:
            message_choice = ['暇だなー','何話そうかな？','うーん']
            message =  random.choice(message_choice)
            steat = "&speaker=hikari"

        #resp = chat.send_and_get(message, steat)
        f = open("/home/paola/openjtalk/index.txt",'w')
        f.write(message)
        f.close()
        cmd = "open_jtalk -m /usr/share/hts-voice/nitech-jp-atr503-m001/nitech_jp_atr503_m001.htsvoice -x /usr/local/dic -ow /home/paola/openjtalk/hhh.wav /home/paola/openjtalk/index.txt "
        subprocess.call(cmd, shell=True)
        """
        test = wave.Wave_write("test.wav")
        test.setnchannels(1)
        test.setsampwidth(2)
        test.setframerate(44100)
        test.writeframes(resp.read())
        test.close()
        wavfile = "test.wav"
        """
        os.system("aplay /home/paola/openjtalk/hhh.wav")
        message = ""
        steat = ""
        
if __name__ == '__main__':
    main()
