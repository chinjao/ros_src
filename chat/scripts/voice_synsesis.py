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
import requests
from numpy import *
from xml.etree.ElementTree import *
import os, glob, random

APP_URL = 'https://api.apigw.smt.docomo.ne.jp/aiTalk/v1/textToSpeech'
 
class DocomoChat(object):
    u"""Docomoの雑談対話APIでチャット"""
    def __init__(self, api_key):
        super(DocomoChat, self).__init__()
        self.api_url = APP_URL + '?APIKEY=%s'%(api_key)
        self.context, self.mode = None, None
    
    def __send_message(self, input_message='' , pitch = '', range = '', rate = '', volume = '',custom_dict = None):

	String = '<?xml version="1.0" encoding="utf-8" ?><speak version="1.1"><voice name="nozomi">わたしはだれ</voice></speak>'
	#data = String.format(input_message)	
	request = urllib2.Request(self.api_url, String)
        request.add_header('Content-Type', 'application/ssml+xml')
        request.add_header('Accept', 'audio/L16')
        request.add_header('Content-Length', '241')
#	print data
        try:
	    response = requests.post(self.api_url,data = String,headers = request.headers)
        except Exception as e:
            print e
            sys.exit()
        return response

    def send_and_get(self, input_message, input_pitch, input_range, input_rate, input_volume):
        response = self.__send_message(input_message,input_pitch,input_range,input_rate,input_volume)
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
    pitch = raw_input('pitch input : ')
    range = raw_input('range input : ')
    rate = raw_input('rate input : ')
    volume = raw_input('volume input : ')
    resp = chat.send_and_get(message,pitch,range,rate,volume)
    write_wave = wave.Wave_write("aaa.wav")
    write_wave.setnchannels(1)
    write_wave.setsampwidth(2)
    write_wave.setframerate(16000)
    write_wave.writeframes(resp.text)
    write_wave.close()
    wavfile = "aaa.wav"
    #os.system("aplay " + wavfile)
if __name__ == '__main__':
    main()
