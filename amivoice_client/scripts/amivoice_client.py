#!/usr/bin/env python
# -*- coding: utf-8 -*-
# license removed for brebity
import time
import re
import socket

host = '133.19.23.53'
port = 9999
def talker() :
        while True:
                clientsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                clientsock.connect((host,port))
                rcvmsg = clientsock.recv(1024)
                if rcvmsg != "":
                        print rcvmsg
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException: pass
	
