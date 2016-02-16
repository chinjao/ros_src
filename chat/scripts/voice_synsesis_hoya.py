#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
u"""Docomoの雑談対話APIを使ってチャットできるスクリプト
"""

import sys
import urllib2
import json
import subprocess
import codecs 
import wave
import pyaudio
from numpy import *
import os, glob, random


APP_URL = 'https://api.apigw.smt.docomo.ne.jp/voiceText/v1/textToSpeech'
 
class DocomoChat(object):
    u"""Docomoの雑談対話APIでチャット"""
    def __init__(self, api_key):
        super(DocomoChat, self).__init__()
        self.api_url = APP_URL + '?APIKEY=%s'%(api_key)
        self.context, self.mode = None, None
    
    def __send_message(self, input_message='' , emotion = '', strange = '', custom_dict = None):
        if emotion == '4':
            req_data = "text=" + input_message + "&speaker=takeru"
        elif emotion == '3':
            req_data = "text=" + input_message + "&speaker=takeru" + "&emotion=sadness" + "&emotion_level=" + strange 
        elif emotion == '2':
            req_data = "text=" + input_message + "&speaker=takeru" + "&emotion=happiness" + "&emotion_level=" + strange 
        elif emotion == '1':
            req_data = "text=" + input_message + "&speaker=takeru" + "&emotion=anger" + "&emotion_level=" + strange 
        print req_data
        request = urllib2.Request(self.api_url, req_data)
        request.add_header('Content-Type', 'application/x-www-form-urlencoded')
        try:
            response = urllib2.urlopen(request)
        except Exception as e:
            print e
            sys.exit()
        return response
 
    def __process_response(self, response):
        resp_json = json.load(response)
        return resp_json['score']

    def send_and_get(self, input_message, emotion, strange):
        response = self.__send_message(input_message,emotion,strange)
        #received_message = self.__process_response(response)
        return response
 
 
def main():
    api_key = '72386c696d44534c33387871674c4a584b4d2e586b627745366c314c64744479307535732f35355054472e'
    chat = DocomoChat(api_key)
    message = ''
    before = u''
    line2 = u''
    end = u"さようなら"
    end.encode('utf-8')
    print "音声合成を行います"
    message = raw_input('語句を入力してください : ')
    emotion = raw_input('どの感情で合成しますか, 1:怒り 2:喜び 3:悲しみ 4:通常 :')
    if emotion == '1':
        strenge = raw_input('感情の強さを入力してください(2段階) :')
    elif emotion == '2':
        strenge = raw_input('感情の強さを入力してください(2段階) :')
    elif emotion == '3':
        strenge = raw_input('感情の強さを入力してください(2段階) :')
    else:
        emotion = '4'
        strenge = '1'
    resp = chat.send_and_get(message,emotion,strenge)
    print type(resp.read())
    print type(resp)
    write_wave = wave.Wave_write("aaa.wav")
    write_wave.setnchannels(1)
    write_wave.setsampwidth(2)
    write_wave.setframerate(44100)
    write_wave.writeframes(resp.read())
    write_wave.close()
    wavfile = "aaa.wav"
    #os.system("aplay " + wavfile)
if __name__ == '__main__':
    main()
