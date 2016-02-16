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

CHAT_URL = 'https://api.apigw.smt.docomo.ne.jp/dialogue/v1/dialogue'
VOICE_URL = 'https://api.apigw.smt.docomo.ne.jp/voiceText/v1/textToSpeech'
 
class DocomoVoice(object):
    u"""Docomoの雑談対話APIでチャット"""
    def __init__(self, api_key):
        super(DocomoVoice, self).__init__()
        self.api_url = VOICE_URL + '?APIKEY=%s'%(api_key)
        self.context, self.mode = None, None
    
    def __send_message(self, input_message='' , emotion = '', strange = '', custom_dict = None):
        req_data = "text=" + input_message + "&speaker=takeru"
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
 
 

class DocomoChat(object):
    u"""Docomoの雑談対話APIでチャット"""
    def __init__(self, api_key):
        super(DocomoChat, self).__init__()
        self.api_url = CHAT_URL + '?APIKEY=%s'%(api_key)
        self.context, self.mode = None, None
    
    def __send_message(self, input_message='', custom_dict = None):
        req_data = {'utt': input_message}
        if self.context:
            req_data['context'] = self.context
        if self.mode:
            req_data['mode'] = self.mode
        if custom_dict:
            req_data.update(custom_dict)
        request = urllib2.Request(self.api_url, json.dumps(req_data))
        request.add_header('Content-Type', 'application/json')
        try:
            response = urllib2.urlopen(request)
        except Exception as e:
            print e
            sys.exit()
        return response
 
    def __process_response(self, response):
        resp_json = json.load(response)
        self.context = resp_json['context'].encode('utf-8')
        self.mode    = resp_json['mode'].encode('utf-8')
        return resp_json['utt'].encode('utf-8')
 
    def send_and_get(self, input_message):
        response = self.__send_message(input_message)
        received_message = self.__process_response(response)
        return received_message
 
    def set_name(self, name, yomi,sex,bloodtype,birthdateM,birthdateD,age,place):
        response = self.__send_message(custom_dict={'nickname': name, 'nickname_y': yomi,'sex': sex, 'bloodtype': bloodtype, 'birthdateM': birthdateM, 'birthdateD': birthdateD,'age': age, 'place': place})
        received_message = self.__process_response(response)
        return received_message
    
 
def main():
    api_key = '72386c696d44534c33387871674c4a584b4d2e586b627745366c314c64744479307535732f35355054472e'
    chat = DocomoChat(api_key)
    chat2 = DocomoVoice(api_key)
    resp = chat.set_name('平井', 'ヒライ','男','A',9,21,21,'京都')
    print '相手　 : %s'%(resp)
    message = ''
    before = u''
    line2 = u''
    end = u"さようなら"
    end.encode('utf-8')
    while line2 != end:
        message = raw_input('あなた : ')
            #message = unicode(line2,'utf-8')
            #message.encode('shift-jis')
        resp = chat.send_and_get(message)
        print '相手　 : %s'%(resp)
            #sh = "sh ~/openjtalk/talk.sh \"%s\""%(resp)
            #subprocess.call(sh,shell = True)
        before = line2
        emotion = 1
        strenge = 1
        resp2 = chat2.send_and_get(resp,emotion,strenge)
        write_wave = wave.Wave_write("aaa.wav")
        write_wave.setnchannels(1)
        write_wave.setsampwidth(2)
        write_wave.setframerate(44100)
        write_wave.writeframes(resp2.read())
        write_wave.close()
        wavfile = "aaa.wav"
        os.system("aplay " + wavfile)
if __name__ == '__main__':
    main()
