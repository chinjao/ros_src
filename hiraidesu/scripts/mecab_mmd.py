#coding:utf-8
import MeCab
tagger = MeCab.Tagger("-Ochasen")
before = u''
while True:
	f = open('/home/yhirai/mecab/test.txt')
	line = f.readline()
	while line:
		line2 = line.rstrip()
		line = f.readline()
	f.close
	if before != line2:
		node = tagger.parseToNode(line2)
		while node:
			print "%s %s"% (node.surface,node.feature)
			node = node.next
		before = line2
	#print result
