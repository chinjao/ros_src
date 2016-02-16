#!/usr/bin/env python
# -*- coding: utf-8 -*-
# license removed for brebity
import rospy
import time
import MeCab
import re
from messanger.msg import mecab

def talker() :
	tagger = MeCab.Tagger("-Ochasen")
	before = ''
	ss = u''
	ss2 = u''
	pub = rospy.Publisher('mecab_res', mecab, queue_size = 10)
	rospy.init_node('mecab', anonymous=True)
	r = rospy.Rate(10)
	msg = mecab()
	while True:
		f = open('/home/yhirai/mecab/test2.txt')
		line = f.readline()
		while line:
			line2 = line.rstrip()
			line = f.readline()
		f.close
		if before != line2:
			print line2
			node = tagger.parseToNode(line2)
			while node:
				msg.word = node.surface
				m = re.search('(.*,)(.*,)(.*,)(.*,)(.*,)(.*,)(.*,)(.*,)',node.feature)
				msg.part = m.group(0)
				print "%s %s"% (node.surface,node.feature)
				time.sleep(0.01)
				pub.publish(msg)
				node = node.next
			before = line2
				
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException: pass
	
