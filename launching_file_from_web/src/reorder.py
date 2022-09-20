#!/usr/bin/env python

import sys
import re
import collections
import rospy

def main_reorder() :
  NodeDict = dict()
  NodeIndex = dict()
  nodeString = ""
  node = ""

  infilestream = open('outputnew.txt', 'r')

  for line in infilestream:
    if re.match(r'ROOT*|THEN*|AND*|OR*|WHILE*|PLACE*', line):
      if nodeString != "":
        NodeDict[node] = nodeString
      nodeString =""
      node = re.sub(r':', "", line.strip())
      numb = re.search(r'...$', node).group()
      NodeIndex[numb] = node
      # print(numb)    
    nodeString += '  ' + line


  NodeDict[node] = nodeString
  NodeIndex = collections.OrderedDict(sorted(NodeIndex.items()))
  temp = int((len(NodeIndex)+1)/2 + 1)
  temp_s = '0'+str(temp)
  temp_s = NodeIndex[temp_s]
  # temp_s = NodeIndex[temp_s] + '_state'
  print(temp_s)
  # print(NodeIndex[temp_s])
  rospy.set_param('/topic', temp_s)
  infilestream.close()


  listString = ("NodeList: [")
  nodeString= "Nodes: \n"
  for key, val in NodeIndex.items():
    nodeString += NodeDict[val]
    listString += "'"+ val +"'" + ", "
  listString = re.sub (r'..$', ' ]\n', listString)

  outfilestream = open('output.yaml', 'w')
  outfilestream.write(listString)
  outfilestream.write(nodeString)
  outfilestream.close()

# if __name__ == '__main__':

#   main()