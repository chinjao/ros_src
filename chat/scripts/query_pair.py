#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
u"""Docomoの雑談対話APIを使ってチャットできるスクリプト
"""
 
import sys
import urllib2
import json
import subprocess
import codecs 

APP_URL = 'https://api.apigw.smt.docomo.ne.jp/gooLanguageAnalysis/v1/similarity'
 
class DocomoChat(object):
    u"""Docomoの雑談対話APIでチャット"""
    def __init__(self, api_key):
        super(DocomoChat, self).__init__()
        self.api_url = APP_URL + '?APIKEY=%s'%(api_key)
        self.context, self.mode = None, None
    
    def __send_message(self, input_message='',input_message2='', custom_dict = None):
        req_data = {'request_id': "record001","query_pair" : [input_message,input_message2]}
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
        return resp_json['score']

    def send_and_get(self, input_message,input_message2):
        response = self.__send_message(input_message,input_message2)
        received_message = self.__process_response(response)
        return received_message
 
 
def main():
    api_key = '594e2e4b5162466d4b555475415a594b7755556765386579707159424577674d4e5737445a375943436742'
    chat = DocomoChat(api_key)
    message = ''
    before = u''
    line2 = u''
    end = u"さようなら"
    end.encode('utf-8')
    print "語句がどれだけ似ているかを判定します"
    message = raw_input('一つ目の語句を入力してください : ')
    message2 = raw_input('二つ目の語句を入力してください : ')
    resp = chat.send_and_get(message,message2)
    print resp
if __name__ == '__main__':
    main()
