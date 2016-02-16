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
	before = ''
	ss = u''
	ss2 = u''
	pub = rospy.Publisher('mecab_res', Speech, queue_size = 1000)
	rospy.init_node('mecab', anonymous=True)
	r = rospy.Rate(10)
	while True:
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
				#print msg.word[i]
				#print msg.word[i]
				#speech_msgs.word[i] = node.surface
				m = re.search('(.*,)(.*,)(.*,)(.*,)(.*,)(.*,)(.*,)(.*,)',node.feature)
				if m == None:
					m = re.search('(.*,)(.*,)(.*,)(.*,)(.*,)(.*,)',node.feature)	
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
				node = node.next
				if word.part[0] != 'BOS/EOS':
					x.words.append(word)
					i += 1
			x.word_num = i;
			print i	
			x.header.stamp = rospy.get_rostime()
			pub.publish(x)
				
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException: pass
	
