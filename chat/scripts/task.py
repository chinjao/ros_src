#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
u"""Docomoの雑談対話APIを使ってチャットできるスクリプト
"""
 
import sys
import urllib2
import json
import subprocess
import codecs 

APP_URL = 'https://api.apigw.smt.docomo.ne.jp/sentenceUnderstanding/v1/task'
api_key = '49337978393057785052794f434d416a4f7442326f3473436d69774c54486470534e545958347a464c7334'

 
class DocomoChat(object):
    u"""Docomoの発話理解APIでチャット"""
    def __init__(self, api_key):
        super(DocomoChat, self).__init__()
      	self.api_url = APP_URL + '?APIKEY=%s'%(api_key)
        self.context, self.mode = None, None
    
    def __send_message(self, input_message='', custom_dict = None):
        req_data = {'projectKey': 'OSU'}
	req_data['appInfo'] = {'appName':'hoge_app'}
	req_data['appInfo']['appKey'] = 'hoge_app01'
	req_data['clientVer'] = '1.0.0'
	req_data['dialogMode'] = 'off'
	req_data['language'] = 'ja'
	req_data['userUtterance'] = {'utteranceText' : input_message}
	print req_data
	#print req_data
	#self.q = input_message
	#self.api_url = APP_URL + '?APIKEY=%s'%(api_key) + '&q=%s'%(urllib2.quote(input_message))
        request = urllib2.Request(self.api_url,req_data)
        request.add_header('Content-Type', 'application/x-www-form-urlencoded')
        try:
            response = urllib2.urlopen(request)
        except Exception as e:
            print e
            sys.exit()

        return response
 
    def __process_response(self, response):
        resp_json = json.load(response)
        return resp_json#.encode('utf8')
 
    def send_and_get(self, input_message):
        response = self.__send_message(input_message)
        received_message = self.__process_response(response)
        return received_message
 
def main():
    chat = DocomoChat(api_key)
    message = ''
    before = u''
    line2 = u''
    end = u"さようなら"
    end.encode('utf-8')
    message = raw_input('質問を入力してください : ')
    resp = chat.send_and_get(message)
    #print resp
    if resp['answers'] :
    	print resp['answers'][0]['answerText']
if __name__ == '__main__':
    main()
