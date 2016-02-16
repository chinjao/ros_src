#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
u"""Docomoの雑談対話APIを使ってチャットできるスクリプト
"""
 
import httplib, urllib, base64
from PIL import Image
import json


########### Python 2.7 #############


headers = {
    # Request headers
    'Content-Type': 'application/octet-stream',
#    'Ocp-Apim-Subscription-Key': '69a7b286ddb4466bb395439b57325fff',
}


params = urllib.urlencode({
    # Request parameters
    'subscription-key': '69a7b286ddb4466bb395439b57325fff',
    'analyzesFaceLandmarks': 'true',
    'analyzesAge': 'true',
    'analyzesGender': 'true',
    'analyzesHeadPose': 'true',
})

try:
    body = ""

    f = open("detection2.jpg","rb")
    #print img
    body = f.read()
    f.close()
    conn = httplib.HTTPSConnection('api.projectoxford.ai')
    conn.request("POST", "/face/v0/detections?%s" % params, body, headers)
    response = conn.getresponse()
    data = response.read()
    print(data)
    conn.close()
except Exception as e:
    print("[Errno {0}] {1}".format(e.errno, e.strerror))


headers2 = {
    # Request headers
    'Content-Type': 'application/octet-stream',
    'Ocp-Apim-Subscription-Key': '364e6c00eb0349649d42218192d6c226',
}

params2 = urllib.urlencode({
    # Request parameters
    'faceRectangles': '{string}',
})

conn2 = httplib.HTTPSConnection('api.projectoxford.ai')
conn2.request("POST", "/emotion/v1.0/recognize?%s" % params2, body, headers2)
response2 = conn2.getresponse()
data2 = response2.read()
print(data2)
conn2.close()



