#!/usr/bin/env python
# -*- coding: utf-8 -*-
# license removed for brebity
import time
import MeCab
import re
def talker() :
	tagger = MeCab.Tagger("-Ochasen")
        f = open('utf8.dic')
        i = 0
        for line in f:
                m = re.search('(.*:)(.*:)(.*:)(.*)',line)
                if i == 0:
                        dict = {m.group(1).rstrip(':'):m.group(4).rstrip(':')}
                        i = 1
                elif i == 1:
                        dict[m.group(1).rstrip(':')] = m.group(4).rstrip(':')
        while True:
                all_point = 0
                sentence = raw_input('prease word:')
                node = tagger.parseToNode(sentence)
                while node:
                        ll = 0
                        m = re.search('(.*,)(.*,)(.*,)(.*,)(.*,)(.*,)(.*,)(.*,)',node.feature)
                        if m == None:
                                m = re.search('(.*,)(.*,)(.*,)(.*,)(.*,)(.*,)',node.feature)
                                ll = 1
                        if ll == 0:
                                word = m.group(7).rstrip(',')
                                torf = word in dict
                        elif ll == 1:
                                word = node.surface
                                torf = node.surface in dict
                        if torf == True:
                                print 'word:{0} positive point:{1}'.format(node.surface,dict[word])
                                all_point += float(dict[word])
                        node = node.next
                print
                print 'all positive point:{0}'.format(all_point)
                if all_point > 0:
                        print 'ポジティブな文ですね'
                elif all_point < 0:
                        print 'ネガティブな文ですね'
                else:
                        print '普通な文ですね'
                print
                        
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException: pass
	
