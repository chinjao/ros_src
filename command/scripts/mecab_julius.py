#!/usr/bin/env python
# -*- coding: utf-8 -*-
# license removed for brebity
import rospy
import time
import MeCab
import re
import socket
host = '133.19.23.117'
port = 10500
from messanger.msg import mecab
def talker() :
	clientsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	clientsock.connect((host,port))
	tagger = MeCab.Tagger("-Ochasen")
	before = ''
	ss = u''
	ss2 = u''
	ok = 0
	line2 = ''
	pub = rospy.Publisher('mecab_res', mecab, queue_size = 1000)
	rospy.init_node('mecab', anonymous=True)
	r = rospy.Rate(10)
	msg = mecab()
	while True:
		"""
		f = open('/home/yhirai/mecab/test2.txt')
		line = f.readline()
		while line:
			line2 = line.rstrip()
			line = f.readline()
		f.close
		"""
		rcvmsg = clientsock.readline().strip()
	#	rcvmsg = clientsock.readline()
	#	rcvmsg = rcvmsg.readline().strip
		print '%s' % (rcvmsg)
	#	line = re.search('WHYPO WORD=?\"(.*)\".*',rcvmsg) 
		line = re.search('WORD=\"([^\"]+)\"',rcvmsg)

		ok1 = re.search('(<RECOGOUT>)',rcvmsg)
		ok2 = re.search('(</RECOGOUT>)',rcvmsg)
	#	line2 = line3.group(1)
	#	line2 = line.group(1)
	#	print line.group(1)
		if line != None:
		#	line2 = re.search('WORD=\"([^\"]+)\",rcvmsg',rcvmsg)
		#       print line.group(1)
			line2 += line.group(1)
			print line.group(0)
		#	print rcvmsg
		elif ok1 != None:
			print 'ok'
		elif ok2 != None:
		#	line2 = line2.rstrip()
		#	print line2
			ok = 1
	#	if before != line2:
		if ok == 2:
			print line2
			node = tagger.parseToNode(line2)
			msg.all = line2
			i = 0
			while node:
				msg.word[i] = node.surface
				m = re.search('(.*,)(.*,)(.*,)(.*,)(.*,)(.*,)(.*,)(.*,)',node.feature)
				if m == None:
					m = re.search('(.*,)(.*,)(.*,)(.*,)(.*,)(.*,)',node.feature)	
				msg.part[i] = m.group(1).rstrip(',')
				print "%s %s"% (node.surface,node.feature)
				if m.group(1)  == '動詞,':
					msg.part2[i] = m.group(2).rstrip(',')
					msg.part3[i] = m.group(6).rstrip(',')
					msg.part4[i] = m.group(7).rstrip(',')
				elif m.group(1) == '名詞,':
					msg.part2[i] = m.group(2).rstrip(',')
				#	if m.group(3) == '人名,':
					msg.part3[i] = m.group(3).rstrip(',')
			#		msg.part4[i] = m.group(7).rstrip(',')
				#time.sleep(0.05)
				node = node.next
				i += 1
			msg.number = i-2;
			pub.publish(msg)
			ok = 0
			line2 = ""
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException: pass
	
