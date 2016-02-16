#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
u"""Docomoの雑談対話APIを使ってチャットできるスクリプト
"""
 
import sys
import urllib2
import json
import subprocess
import codecs 

APP_URL = 'https://api.apigw.smt.docomo.ne.jp/gooLanguageAnalysis/v1/entity'
 
class DocomoChat(object):
    u"""Docomoの雑談対話APIでチャット"""
    def __init__(self, api_key):
        super(DocomoChat, self).__init__()
        self.api_url = APP_URL + '?APIKEY=%s'%(api_key)
        self.context, self.mode = None, None
    
    def __send_message(self, input_message='', custom_dict = None):
        req_data = {'request_id': "record001","sentence" : input_message}
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
        return resp_json['ne_list']#.encode('utf-8')
 
    def send_and_get(self, input_message):
        response = self.__send_message(input_message)
        received_message = self.__process_response(response)
        return received_message
 
    def set_name(self, name, yomi,sex,bloodtype,birthdateM,birthdateD,age,place):
        response = self.__send_message(custom_dict={'nickname': name, 'nickname_y': yomi,'sex': sex, 'bloodtype': bloodtype, 'birthdateM': birthdateM, 'birthdateD': birthdateD,'age': age, 'place': place})
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
    message = raw_input('文章を入力してください : ')
    resp = chat.send_and_get(message)
    for num in range(0,len(resp)):
        print resp[num][0].encode('utf-8'),
        print ':',
        print resp[num][1].encode('utf-8')
if __name__ == '__main__':
    main()
