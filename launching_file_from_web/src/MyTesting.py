
#%%
import numpy as np
from anytree import Node, RenderTree

andDict = dict()
orDict = dict()
thenDict = dict()

############ HELPER FUNCTIONS ############

# find all OR nodes (aka any node in ex1 that is not in ex2 and visa versa)
def findOrNodes(ex1, ex2):
    global orDict

    orNodes = np.empty((0,5), int)
    for i in ex1:
        if not np.isin(i, ex2):
            orNodes = np.append(orNodes,i)
    for i in ex2:
        if not np.isin(i, ex1):
            orNodes = np.append(orNodes,i)
    orNodes.sort()
    for i in range(0, orNodes.size-1,2):
        node = Node("OR")
        left = Node(orNodes[i], parent=node)
        right = Node(orNodes[i+1], parent=node)
        orDict[orNodes[i]] = node

    return orNodes

# replaces OR node value from ex2 with pairs (from ex1)
def replaceOrNodes(ex, orNodes, ex1):
    if not len(ex) <= 2 and not len(ex1) <= 2:
        for i in range(0,orNodes.size-1):
            ex = np.where(ex==orNodes[i+1], int(orNodes[i]), ex)
    return ex

def initGraph(ex1, ex2):
    # initialize graph with first elements coming from 0 start node
    graph = dict()
    graph[0] = np.array([ex1[0],ex2[0]])
    # add each outgoing node from its previous root node
    for i in range(0,ex1.size-1):
        if ex1[i] in graph:
            graph[ex1[i]] = np.append(graph[ex1[i]], ex1[i+1])
        else:
            graph[ex1[i]] = np.array([ex1[i+1]])
    for i in range(0,ex2.size-1):
        if ex2[i] in graph:
            graph[ex2[i]] = np.append(graph[ex2[i]], ex2[i+1])
        else:
            graph[ex2[i]] = np.array([ex2[i+1]])
    return graph

def findAndNodes(graph):
    global andDict
    andNodes = np.empty((0,2), int)
    # find pairs in graph where they exist in each others dest.
    for key in graph:
        for i in graph[key]:
            if int(i) in graph and key in graph[int(i)]:
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

# looks for THEN nodes (there will be 2 of the same vals for a key)
def findThenNodes(graph, andNodes, orNodes):
    global thenDict
    thenNodes = np.empty((0,2), int)
    for key in graph:
        keyVals = np.empty((0,3), int)
        for i in graph[key]:
            if i > 0 and key > 0:
                if not np.isin(i, keyVals):
                    keyVals = np.append(keyVals, i)
                elif not np.isin(i, andNodes) and not np.isin(key, andNodes):
                    if not np.isin(i, orNodes) and not np.isin(key, orNodes):
                        node = Node("THEN")
                        if i < key:
                            thenNodes = np.concatenate((thenNodes, [[i, key]]), axis=0)
                            left = Node(i, parent=node)
                            right = Node(key, parent=node)
                        else:
                            thenNodes = np.concatenate((thenNodes, [[key, i]]), axis=0)
                            left = Node(key, parent=node)
                            right = Node(i, parent=node)
                        if i not in thenDict:
                            thenDict[i] = node
    return thenNodes

# looks for larger AND relationships
def andBuilding(ex1, ex2):
    global andDict
    global orDict
 
    delKey1 = delKey2 = 10
    for key in andDict:
        index1 = index2 = index3 = index4 = 0
        for i in andDict[key].children: 
            for j in range(0,ex1.size):
                if j < len(ex1) and j < len(ex2): # added this in case of diff length examples
                    if ex1[j] == i.name:          # don't want to go out of bounds
                        index1 += j
                    if ex2[j] == i.name:
                        index2 += j
        if index1 != index2:
            newNode = Node("AND")
            newLeft = andDict[key]
            newLeft.parent = newNode
            if (index1 - index2) == 2 or (index2 - index1) == 2:
                for key2 in orDict:
                    for i2 in range(0,ex1.size):
                        if ex1[i2] == key2:
                            index3 += i2
                        if ex2[i2] == key2:
                            index4 += i2
                    if index3 != index4:
                        if (index3 - index4) == 2 or (index4 - index3) == 2:
                            newRight = orDict[key2]
                            newRight.parent = newNode
                            delKey2 = key2        
                            break
            else:
                if (index1 - index2) == 4 or (index2 - index1) == 4:
                    nextKey = key+2
                    nextKey2 = key+1
                    lastKey = key-2
                    if nextKey in andDict:
                        newRight = andDict[nextKey]
                        newRight.parent = newNode
                        delKey1 = nextKey
                    elif nextKey2 in andDict:
                        newRight = andDict[nextKey2]
                        newRight.parent = newNode
                        delKey1 = nextKey2
                    elif lastKey in andDict:
                        newRight = andDict[lastKey]
                        newRight.parent = newNode
                        delKey1 = lastKey
            andDict[key] = newNode
            break
    if delKey1 != 10:
        del andDict[delKey1]
    elif delKey2 != 10:
        del orDict[delKey2]
    if len(andDict.keys()) > 2:
        ex1, ex2 = andBuilding(ex1, ex2)

    return ex1, ex2

def swapBack(ex, orNodes, ex1):
    if not len(ex) <= 2 and not len(ex1) <= 2:
        for i in range(0,orNodes.size-1):
            if np.isin(orNodes[i], ex) and np.isin(orNodes[i], ex1):
                if np.isin(orNodes[i+1], ex):
                    if np.isin(orNodes[i+1]+1, orNodes):
                        ex = np.where(ex==orNodes[i+1], int(orNodes[i+2]), ex)
                ex = np.where(ex==orNodes[i], int(orNodes[i+1]), ex)
    return ex

def orBuilding(ex1, ex2, orNodes, andNodes):
    global andDict
    global orDict
    
    delKey = delKey2 = 10
    keys = [0] * 4
    if len(ex1) != len(ex2):
        orDict.clear()
        for i in range(0, orNodes.size-1):
            if ((np.isin(orNodes[i], ex1) and np.isin(orNodes[i+1], ex1)) or 
                        (np.isin(orNodes[i], ex2) and np.isin(orNodes[i+1], ex2))):
                newNode = Node("OR")
                node = Node("AND", parent=newNode)
                left = Node(orNodes[i+1], parent=node)
                right = Node(orNodes[i], parent=node)
                orDict[orNodes[i]] = newNode
                delKey = i
        if delKey != 10:
            keys[0] = orNodes[delKey]
            keys[1] = orNodes[delKey+1]
            orNodes = np.delete(orNodes, delKey+1)
            orNodes = np.delete(orNodes, delKey)
            dKey = 10
            for key in andDict:
                for value in andDict[key].children:
                    if keys[0] == value.name or keys[1] == value.name:
                        dKey = key
            if dKey != 10:
                del andDict[dKey]
        for i in range(0, orNodes.size-1):
            if orNodes[i+1] - orNodes[i] == 1:
                node = Node("OR")
                left = Node(orNodes[i], parent=node)
                right = Node(orNodes[i+1], parent=node)
                orDict[orNodes[i]] = node
                delKey2 = i
        if delKey2 != 10:
            keys[2] = orNodes[delKey2]
            keys[3] = orNodes[delKey2+1]
            orNodes = np.delete(orNodes, delKey2+1)
            orNodes = np.delete(orNodes, delKey2)
        for i in orNodes:
            if not i-1 in orDict:
                if not np.isin(i-1, keys) and not np.isin(i-1, andNodes) and i-1 > 0:
                    node = Node("OR", parent=orDict[keys[0]])
                    left = Node(i-1, parent=node)
                    right = Node(i, parent=node)
                else:
                    node = Node("OR", parent=orDict[keys[0]])
                    left = Node(i, parent=node)
                    right = Node(i+1, parent=node)
    else:
        for key in orDict:
            orPairs = orDict[key].children
            if orPairs[1].name - orPairs[0].name != 1:
                newNode = Node("OR")
                node1 = Node("OR", parent=newNode)
                left = Node(orPairs[0].name, parent=node1)
                right = Node((orPairs[0].name)+1, parent=node1)
                node2 = Node("OR", parent=newNode)
                left2 = Node(orPairs[1].name, parent=node2)
                if (orPairs[1].name)-1 != (orPairs[0].name)+1:
                    right2 = Node((orPairs[1].name)-1, parent=node2)
                else:
                    right2 = Node((orPairs[1].name)+1, parent=node2)
                orDict[orPairs[0].name] = newNode
        if len(orDict.keys()) == 3 or len(andDict.keys()) == 1:
            newNode = Node("OR")
            for i in range(0, orNodes.size-1):
                if np.isin(orNodes[i], ex1) and np.isin(orNodes[i+1], ex1):
                    node = Node("AND", parent=newNode)
                    left = Node(orNodes[i], parent=node)
                    right = Node(orNodes[i+1], parent=node)
                    delKey = i
                    i += 1
                elif np.isin(orNodes[i], ex2) and np.isin(orNodes[i+1], ex2):
                    node = Node("AND", parent=newNode)
                    left = Node(orNodes[i], parent=node)
                    right = Node(orNodes[i+1], parent=node)
                    delKey2 = i
            if delKey != 10:
                orDict[orNodes[delKey]] = newNode
            if delKey2 != 10:
                if orNodes[delKey2] in orDict:
                    del orDict[orNodes[delKey2]]
                else:
                    del orDict[orNodes[delKey2+1]]
    return orNodes

def reconstruct(andNodes, orNodes, ex1):
    global andDict
    global orDict
    global thenDict
    print(andDict)
    tree = Node("THEN")

    inTree = np.empty((0,8), int)
    for key in andDict:
        for key2 in orDict:
            if key > key2 and not np.isin(key2, andNodes) and not np.isin(key2, inTree):
                node = orDict[key2]
                node.parent = tree
                for child in orDict[key2].children:
                    if any(child.children):
                        for child2 in child.children:
                            inTree = np.append(inTree, child2.name)
                    else:
                        inTree = np.append(inTree, child.name)
            elif np.isin(key, orNodes) or np.isin(key2, andNodes) and not np.isin(key2, inTree):
                parentNode = Node("AND")
                if key in orDict:
                    node1 = orDict[key]
                    for child in orDict[key].children:
                        inTree = np.append(inTree, child.name)
                else:
                    node1 = orDict[key2]
                    for child in orDict[key2].children:
                        inTree = np.append(inTree, child.name)
                node1.parent = parentNode
                for i in andNodes:
                    if not np.isin(i, orNodes) and not i in orDict and not np.isin(i, inTree) and i<=key+1:
                        if ((key - i) <=2 and (key - i) > 0) or ((i - key) <= 2 and (i - key) > 0):
                            node2 = Node("AND", parent=parentNode)
                            left = Node(i, parent=node2)
                            j = i-1
                            k = i+1
                            if not np.isin(j, orNodes) and j > 0:
                                right = Node(j, parent=node2)
                                inTree = np.append(inTree, j)
                            if not np.isin(k, orNodes) and k < 9:
                                right = Node(k, parent=node2)
                                inTree = np.append(inTree, k)
                            parentNode.parent = tree
                            inTree = np.append(inTree, i)
                    elif i in orDict and not np.isin(i, inTree) and i <= key+1:
                        node2 = orDict[i]
                        node2.parent = parentNode
                        parentNode.parent = tree
                        for child in orDict[i].children:
                            inTree = np.append(inTree, child.name)
                break
            elif not np.isin(key, inTree):
                node = andDict[key]
                node.parent = tree
                inTree = np.append(inTree, key)
    for val in inTree:
        if int(val) in andDict:
            del andDict[val]
        if int(val) in orDict:
            del orDict[val]
    dk = dk2 = 10
    if any(andDict) and any(orDict):
        for key in andDict:
            for key2 in orDict:
                parentNode = Node("AND")
                node1 = orDict[key]
                node1.parent = parentNode
                dk = key
                for i in andNodes:
                    if i in orDict:
                        node2 = orDict[i]
                        node2.parent = parentNode
                        parentNode.parent = tree
                        dk2 = i
                        break
                break
    if dk != 10:
        del andDict[dk]
        del orDict[dk]
    if dk2 != 10 and dk2 in orDict:
        del orDict[dk2]
    delKey1 = delKey2 = 10
    for key in thenDict:
        if any(orDict):
            for key2 in orDict:
                if key > key2:
                    node = orDict[key2]
                    node.parent = tree
                    delKey1 = key2
        if any(andDict):
            for key3 in andDict:
                if key > key3:
                    node = andDict[key3]
                    node.parent = tree
                    delKey2 = key3
        node = thenDict[key]
        node.parent = tree
    if delKey1 != 10:
        del orDict[delKey1]
    if delKey2 != 10:
        del andDict[delKey2]
    for key in orDict:
        print(key)
        node = orDict[key]
        node.parent = tree
        print(node)
    for key in andDict:
        node = andDict[key]
        node.parent = tree
    

    return tree
        
def mainAlg(ex1, ex2):
    orNodes = findOrNodes(ex1, ex2)
    print(orNodes)
    ex2 = replaceOrNodes(ex2, orNodes, ex1)
    print("ex2",ex2)
    graph = initGraph(ex1, ex2)
    print(graph)
    andNodes = findAndNodes(graph)
    thenNodes = findThenNodes(graph, andNodes, orNodes)
    ex1, ex2 = andBuilding(ex1, ex2)
    ex2 = swapBack(ex2, orNodes, ex1)
    orNodes = orBuilding(ex1, ex2, orNodes, andNodes)
    print(orNodes)
    print(andNodes)
    return ex1, ex2, andNodes, orNodes


#%%

### Depth of 3 on both sides ###

# case 1 -- works!!
# ex1 = np.array([1,2,3,4,5])
# ex2 = np.array([4,3,2,1,7])

# case 2 -- works!!
# ex1 = np.array([1,3,2,5,7])
# ex2 = np.array([2,4,1,8,6])

# case 3 -- works!!
# ex1 = np.array([1,5,6])
# ex2 = np.array([4,8,7])

# case 4 -- works!!
# ex1 = np.array([1,5,7,8])
# ex2 = np.array([4,3,8,7,6])

# case 5 -- works!!
# ex1 = np.array([1,5])
# ex2 = np.array([4,8])

# case 6 -- works!!
# ex1 = np.array([1,2,3,4,5,6,7,8])
# ex2 = np.array([4,3,2,1,8,7,6,5])

### THEN and AND ###

# case 1 -- works!!
# ex1 = np.array([1,2,3,4])
# ex2 = np.array([1,2,4,3])

# case 2 -- works!!
# ex1 = np.array([1,2,3,4])
# ex2 = np.array([2,1,3,4])

### THEN and OR ###

# case 1 -- works!!
# ex1 = np.array([1,2,3])
# ex2 = np.array([1,2,4])

# case 2 -- works!!
# ex1 = np.array([1,3,4])
# ex2 = np.array([2,3,4])

### THEN branch ###

# case 1 -- works!!
# ex1 = np.array([1,2,3,4,5,6])
# ex2 = np.array([4,3,2,1,5,6])

# case 2 -- works!!
# ex1 = np.array([1,2,3])
# ex2 = np.array([1,2,6])

### THEN at depth 3 ###

# case 1 -- treats THEN as AND
# ex1 = np.array([1,2,3,5])
# ex2 = np.array([4,1,2,6])

# AND-AND/AND-AND/AND, OR
# ex1 = np.array([1,3,6,4,5])
# ex2 = np.array([2,3,5,4,6])

# print("ex1: " + str(ex1))
# print("ex2: " + str(ex2))

# global andDict

# ex1, ex2, andNodes, orNodes = mainAlg(ex1, ex2)

# andNodes = andNodes.flatten()
# tree = reconstruct(andNodes, orNodes, ex1)
# print("\n\nRECONSTRUCTED TREE: \n")
# print(RenderTree(tree))


#%%
