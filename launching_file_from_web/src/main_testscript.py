#!/usr/bin/env python3

# Program for taking sequence_builder.cc output and inputting to main_documented.py

# Output files look like this:
# 
# task sequence key:
# 1 = yellow
# 2 = blue 
# [13456]
# [26543]
#
# and the name of the file is given by the user

import numpy as np
from anytree import Node, RenderTree, PreOrderIter
from MyTesting import *
from parse_task_tree import *
from reorder import *
from publishing_command import *

#object = ["one","two","three","four","five","six"]
object = ["Cup","Tea","Sugar","Bread1","Bread2","Meat","lettuce"]

def main():
    # user enters output file name 
    # file_name = input("Enter sequence_builder.cc output file name: ")

    # looks through file for sequences
    file_name = "sequence.txt"
    sequences = []
    with open(file_name) as f:
        for line in f:
            if '[' in line:
                temp = line.strip()
                sequences.append(temp.strip('[]'))
    f.close()

    # constructing first sequence 
    seq1 = []
    temp = sequences[0]
    for x in temp:
        seq1.append(int(x))
    ex1 = np.array(seq1)

    # constructing second sequence
    seq2 = []
    temp = sequences[1]
    for x in temp:
        seq2.append(int(x))
    ex2 = np.array(seq2)

    # print sequences to terminal so user can see
    print("ex1: " + str(ex1))
    print("ex2: " + str(ex2))

    # put sequences into algorithm which findes 'AND', 'OR' relationships, etc.
    # while len(ex1) != 2:
    ex1, ex2, andNodes, orNodes = mainAlg(ex1, ex2)
    print("callilng")

    # create hierarchical task tree  
    andNodes = andNodes.flatten()               # 'flattens' array, collapses it into one dimension
    tree = reconstruct(andNodes, orNodes, ex1) 

    # print out results
    print("\n\nRECONSTRUCTED TREE: \n")
    print(RenderTree(tree))
    # for pre, fill, node in RenderTree(tree):
    #     print("%s %s" % (node.name,node.children)) 
    #     if node.is_leaf and (node.name=="AND" or node.name=="OR"):
    #         node.parent=None
    # print("\n\nRECONSTRUCTED TREE: \n")
    # print(RenderTree(tree))


    #       "(THEN "
    # "(AND"
    #     "(PLACE apple) "
    #     "(PLACE teddy_bear)) "
    # "(OR"
    #     "(PLACE clock) "
    #     "(PLACE scissors))) "
    #   )

    str_tree=['(']
    print(str_tree)
    print(tree.root)
    #print([node.name for node in PreOrderIter(tree.root)])
    treeToString(tree.root,str_tree)
    str_tree.append(')')
    str_tree = ''.join(str_tree)
    print(''.join(str_tree))
    main_parse_task_tree(str_tree)

    main_reorder()
    main_publisher()



def treeToString(root: Node, string: list):
    if root.is_leaf:
        string.append("PLACE ")
        string.append(object[int(root.name)-1])
    else:
        string.append(str(root.name))
    print(string)
    if root.is_leaf:
        return
    
    s = [node for node in root.children]
    for x in range(len(s)):
        string.append('(')
        treeToString(s[x], string)
        string.append(')')
    
if __name__ == '__main__':
    main()
