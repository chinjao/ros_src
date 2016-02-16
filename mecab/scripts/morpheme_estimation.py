#!/usr/bin/env python
# -*- coding: utf-8 -*-
# license removed for brebity
import rospy
import time
import MeCab
import re
from std_msgs.msg import String
from messanger.msg import mecab_all
from speech_msgs.msg import Speech
from speech_msgs.msg import Word
def talker() :
	tagger = MeCab.Tagger("-Ochasen")
        f = open('/home/yhirai/negative/utf8.dic')
        i = 0
        for line in f:
                m = re.search('(.*:)(.*:)(.*:)(.*)',line)
                if i == 0:
                        dict = {m.group(1).rstrip(':'):m.group(4).rstrip(':')}
                        i = 1
                elif i == 1:
                        dict[m.group(1).rstrip(':')] = m.group(4).rstrip(':')
	before = ''
	ss = u''
	ss2 = u''
	pub = rospy.Publisher('mecab_res', Speech, queue_size = 1000)
	rospy.init_node('mecab', anonymous=True)
	r = rospy.Rate(10)
	while True:
		all_point = 0
		number = 0
		#f = open('/home/yhirai/mecab/test2.txt')
		x = rospy.wait_for_message("zmq_publish",Speech)
		line2 = x.sentence
		before = "1"
		if before != line2:
			#print line2
			node = tagger.parseToNode(line2)
			i = 0
			#speech_msgs = Speech()
			while node:
				word = Word()
				word.word = node.surface
				ll = 0
				#print msg.word[i]
				#print msg.word[i]
				#speech_msgs.word[i] = node.surface
				m = re.search('(.*,)(.*,)(.*,)(.*,)(.*,)(.*,)(.*,)(.*,)',node.feature)
				if m == None:
					m = re.search('(.*,)(.*,)(.*,)(.*,)(.*,)(.*,)',node.feature)	
					ll = 1
				word.part.append( m.group(1).rstrip(',') )
				print "%s %s"% (node.surface,node.feature)
				if m.group(1)  == '動詞,':
					word.part.append(m.group(2).rstrip(','))
					word.part.append(m.group(6).rstrip(','))
					word.part.append(m.group(7).rstrip(','))
				elif m.group(1) == '名詞,':
					word.part.append(m.group(2).rstrip(','))
				#	if m.group(3) == '人名,':
					word.part.append(m.group(3).rstrip(','))
			#		msg.part4[i] = m.group(7).rstrip(',')
				#time.sleep(0.05)
	                        if ll == 0:
	                                word_e = m.group(7).rstrip(',')
	                                torf = word_e in dict
	                        elif ll == 1:
	                                word_e = node.surface
	                                torf = node.surface in dict
	                        if torf == True:
					print float(dict[word_e])
					if float(dict[word_e]) > 0.7 or float(dict[word_e]) < -0.7:
	         	                	all_point += float(dict[word_e])
						number = number + 1
				node = node.next
				if word.part[0] != 'BOS/EOS':
					x.words.append(word)
					i += 1
			x.word_num = i;
			print i	
			x.header.stamp = rospy.get_rostime()
			if number != 0:
				avg = all_point/number;

#				if all_point/number > 0:
#					x.positive_point = 1
#				elif all_point/number < 0:
#					x.positive_point = -1
#
				if avg == 0:
					x.positive_point = 0
				else:
					x.positive_point = avg
			else:
				x.positive_point = 0
			print x.positive_point
			pub.publish(x)
				
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException: pass
	
