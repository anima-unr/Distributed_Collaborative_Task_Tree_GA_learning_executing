#!/usr/bin/env python
import numpy as np
from anytree import Node, RenderTree

global andDict
andDict = dict()
orDict = dict()

############ HELPER FUNCTIONS ############

# find all OR nodes (aka any node in ex1 that is not in ex2 and visa versa)
def findOrNodes(ex1, ex2):
    global orDict
    orNodes = np.empty((0,3), int)              # uninitialized int array w/1 row, 3 cols
    for i in ex1:
        if not np.isin(i, ex2):                 # if i from ex1 is not in ex2
            orNodes = np.append(orNodes,i)      # add i to orNodes
    for i in ex2:
        if not np.isin(i, ex1):                 # if i from ex2 is not in ex1
            orNodes = np.append(orNodes,i)      # add i to orNodes 
    orNodes.sort()                              # sorts in ascending order, returns nothing
    for i in range(0, orNodes.size-1,2):        # iterates through orNodes, skipping every other int
        node = Node("OR")                       
        left = Node(orNodes[i], parent=node)
        right = Node(orNodes[i+1], parent=node) # OR node is constructed from found pair
        orDict[orNodes[i]] = node               # adds constructed node to orDict
    print(orDict)
    return orNodes

# reduction of OR nodes found in ex1 and ex2; one of the ints is replaced by its pair
def replaceOrNodes(ex, orNodes):
    for i in range(0,orNodes.size-1):
        print(orNodes[i], ex)
        ex = np.where(ex==orNodes[i+1], int(orNodes[i]), ex) # where ex==orNodes[i+1], change int to orNodes[i]
    return ex

def replaceOrNodes_(ex, orNodes):
    print(orNodes)
    for i in range(0,ex.size):
        if ex[i] in orNodes:
            index = np.where(orNodes==ex[i])
            ex[i] = orNodes[int(index[0])+1]
            print(int(index[0]))

    # for i in range(0,orNodes.size):
    #     print(orNodes[i], ex)
    #     ex = np.where(ex==orNodes[i], int(orNodes[i+1]), ex) # where ex==orNodes[i+1], change int to orNodes[i]
    return ex

def initGraph(ex1, ex2):
    # initialize graph with first elements coming from 0 start node
    graph = dict()
    graph[0] = np.array([ex1[0],ex2[0]])
    print(graph) 
    # add each outgoing node from its previous root node
    for i in range(0,ex1.size-1):
        if ex1[i] in graph:                     # 'in' looks for key, not val
            graph[ex1[i]] = np.append(graph[ex1[i]], ex1[i+1])
        else:
            graph[ex1[i]] = np.array([ex1[i+1]])
        print(graph)
    for i in range(0,ex2.size-1):
        if ex2[i] in graph:
            graph[ex2[i]] = np.append(graph[ex2[i]], ex2[i+1])
        else:
            graph[ex2[i]] = np.array([ex2[i+1]])
        print(graph)
    return graph

def findAndNodes(graph):
    global andDict
    andNodes = np.empty((0,2), int)
    # find pairs in graph where they exist in each others dest.
    for key in graph:
        for i in graph[key]:
            if int(i) in graph:
                if key in graph[int(i)]:
                    if key not in andNodes:
                        node = Node("AND")
                        if i < key:
                            andNodes = np.concatenate((andNodes, [[i, key]]), axis=0)  
                            left = Node(i, parent=node)
                            right = Node(key, parent=node)
                        else:
                            andNodes = np.concatenate((andNodes, [[key, i]]), axis=0)
                            left = Node(key, parent=node)
                            right = Node(i, parent=node)
                        if i not in andDict:
                            andDict[i] = node
    return andNodes

def mergeAnds(ex, andNodes):
    andNodesNew = np.array([])
    for i in andNodes:
        print("merge",i[0],i[1])
        index = np.argwhere(ex==i[0])           # gives index (row, col) of where AND nodes are in ex 
        print(index)
        ex = np.delete(ex, index)               # deletes AND nodes from ex using index (to reduce graph)
        andNodesNew = np.append(andNodesNew, int(i[1])) # adds int from AND pair not found to andNodesNew
    print(andNodesNew)
    return ex

def reconstruct(andNodes, orNodes, ex1):
    global andDict
    global orDict
    tree = Node("THEN")
    print(andDict)

    for k in ex1:                                               # goes through ex1 task sequence (should only have 2 vals left)
        if k in andDict:                                        # if val is in andDict
            if len(andDict) == 1 or k not in andNodes:          # assumes 1 left is OR
                andDict[k].parent = tree                        # adds AND pair/node to tree
            else:                                               # more than one AND pair 
                for i in range(0, andNodes.size-1,2):
                    node = Node("AND")
                    if(andNodes[i] in andDict):
                        andDict[andNodes[i]].parent = node
                    elif(andNodes[i] in orDict):
                        orDict[andNodes[i]].parent = node
                    if(andNodes[i+1] in andDict):
                        andDict[andNodes[i+1]].parent = node
                    elif(andNodes[i+1] in orDict):
                        orDict[andNodes[i+1]].parent = node
                    node.parent = tree                          # adds new node to tree
        elif k in orDict:                                       # does same thing for OR
            if len(orDict) == 1 or k not in orNodes:
                orDict[k].parent = tree
            else:
                for i in range(0, orNodes.size-1,2):
                    node = Node("OR", parent = tree)
                    if(orNodes[i] in andDict):
                        andDict[orNodes[i]].parent = node
                    elif(orNodes[i] in orDict):
                        orDict[orNodes[i]].parent = node
                    if(orNodes[i+1] in andDict):
                        andDict[orNodes[i+1]].parent = node
                    elif(orNodes[i+1] in orDict):
                        orDict[orNodes[i+1]].parent = node 
                node.parent = tree                              # might be redundant
        # else:
        #     Node(k, parent=tree)
    return tree
        
def mainAlg(ex1, ex2):

    # orNodes = findOrNodes(ex1, ex2)
    # print("ornodes",orNodes)
    # ex1_index = []
    # ex2_index = []
    # ex1_ = ex1
    # ex2_ = ex2
    # for i in range(0,ex1.size):
    #     if ex1[i] in orNodes:
    #         ex1_index.append(i)
    
    # ex1_ = np.delete(ex1_,ex1_index)
    # for i in range(0,ex2.size):
    #     if ex2[i] in orNodes:
    #         ex2_index.append(i)
    # print(ex1_,ex2_)
    
    # ex2_ = np.delete(ex2_,ex2_index)



    # print(ex1_,ex2_)
    # graph = initGraph(ex1_, ex2_)
    # andNodes = findAndNodes(graph)
    # print("and nodes",andNodes)
    
    # ex2 = replaceOrNodes(ex2, orNodes)
    # print("replaceOrNodes",ex2)
    # # while len(ex1) != 2:
    # ex1 = mergeAnds(ex1, andNodes)
    # ex2 = mergeAnds(ex2, andNodes)
    # print("mergeAnds",ex1,ex2)

    # ex2 = replaceOrNodes_(ex2, orNodes)
    # print("replaceOrNodes",ex2)
    orNodes = findOrNodes(ex1, ex2)
    print("ornodes",orNodes)
    ex2 = replaceOrNodes(ex2, orNodes)
    print("replaceOrNodes",ex2)
    graph = initGraph(ex1, ex2)
    andNodes = findAndNodes(graph)
    print("and nodes",andNodes)

    ex1 = mergeAnds(ex1, andNodes)
    ex2 = mergeAnds(ex2, andNodes)
    print("mergeAnds",ex1,ex2)
    
    return ex1, ex2, andNodes, orNodes

#if __name__ == '__main__':
#    main()
#%%
# case 1 -- works!!
# ex1 = np.array([1,2,3])
# ex2 = np.array([2,1,4])

# case 1.2 -- works!!
# ex1 = np.array([1,3,4])
# ex2 = np.array([2,4,3])

# case 3 -- works!!
# ex1 = np.array([1,2,3,4,5])
# ex2 = np.array([4,3,2,1,6])

# case 3.2 -- works!!
#ex1 = np.array([1,3,4,5,6])
#ex2 = np.array([2,6,5,4,3])

# case 4 -- works!!
# ex1 = np.array([1,2,3,4,5,6])
# ex2 = np.array([4,3,2,1,6,5])

# case 4.2 -- works!!
# ex1 = np.array([1,2,3,4,5,6])
# ex2 = np.array([2,1,6,5,4,3])

# I commented out the code below so that I could test 'Main_testscript.py'
"""

print("ex1: " + str(ex1))
print("ex2: " + str(ex2))

global andDict

while len(ex1) != 2:
    ex1, ex2, andNodes, orNodes = mainAlg(ex1, ex2)
    
andNodes = andNodes.flatten()               # 'flattens' array, collapses it into one dimension
tree = reconstruct(andNodes, orNodes, ex1)
print("\n\nRECONSTRUCTED TREE: \n")
print(RenderTree(tree))                     # anytree feature (Node is too)
"""
