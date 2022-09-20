#!/usr/bin/env python3
import time
import numpy as np

import config
base_rule_length = config.base_rule_length
W_CORR = config.w_corr
W_EXIST = config.w_exist
W_REP = config.w_rep

# MAX_FITNESS = W_CORR*base_rule_length + W_EXIST*base_rule_length
MAX_FITNESS = W_CORR*(len(config.task_strings)+len(config.bad_task_strings)) + W_EXIST*base_rule_length
SAVE_THRESH =  len(config.task_strings)#W_CORR*(len(config.task_strings)-1)#MAX_FITNESS/4.0

CORRECT = -1
NONZERO = -1
REPEATS = -1

orval = config.orval
andval = config.andval
thenval = config.thenval
whileval = config.thenval

last_rule_number = 0
rule_dict = {}
funct_dict = {}
bad_funct_dict = {}
child_dict = {}
constraints_dict = {}

string = []
object_list = ["blue_leg","green_leg","yellow_bar","pink_bar","orange_top","purple_top"]
# Necessary to get latest rule number for the other files for GA!
def get_last_rule_number():
	return last_rule_number
def get_rule_dict():
	return rule_dict
def get_funct_dict():
	return funct_dict
def get_score_comps():
	return (CORRECT, NONZERO, REPEATS, constraints_dict)

def searchDuplicate(list1,list2):
	l1ind = 0
	l2ind = 0
	while l1ind < len(list1) and l2ind < len(list2):
		if list1[l1ind] == list2[l2ind]:
			return True
		if list1[l1ind] > list2[l2ind]:
			l2ind = l2ind + 1
		else:
			l1ind = l1ind + 1
	return False


def get_inds(input_int_array):
	out_array = np.ones((base_rule_length,),dtype=np.int32)*-1
	for a in range(len(input_int_array)):
		out_array[input_int_array[a]] = a
	return out_array

def getBase(ruleNum):
	def baseAssist(array,ruleNum=ruleNum):
		return array,1 if array[ruleNum]>=0 else -1,array[ruleNum],array[ruleNum]
	baseAssist.list_items = [ruleNum]
	baseAssist.duplicates = False
	baseAssist.multiplier = 1
	return baseAssist

def getOr(left_in,right_in):
	def orAssist(array,left=left_in,right=right_in):
		if (left(array)[1] + right(array)[1] != 0) or orAssist.duplicates:
			return array,-1,min(left(array)[2],right(array)[2]),max(left(array)[3],right(array)[3])
		else:
			return array,1,min(left(array)[2],right(array)[2]) if left(array)[2] != -1 and right(array)[2] != -1 else max(left(array)[2],right(array)[2]) ,max(left(array)[3],right(array)[3])
	orAssist.list_items = left_in.list_items + right_in.list_items
	orAssist.list_items.sort()
	orAssist.multiplier = (left_in.multiplier + right_in.multiplier)*orval
	if left_in.duplicates or right_in.duplicates:
		orAssist.duplicates = True
	else:
		orAssist.duplicates = searchDuplicate(left_in.list_items,right_in.list_items)
	return orAssist

def getAnd(left_in,right_in):
	def andAssist(array,left=left_in,right=right_in):
		# print(left(array)[1] , right(array)[1])
		if (left(array)[1] + right(array)[1] <= 0) or andAssist.duplicates:
			return array,-1,min(left(array)[2],right(array)[2]) if left(array)[2] != -1 and right(array)[2] != -1 else max(left(array)[2],right(array)[2]) ,max(left(array)[3],right(array)[3])
		else:
			return array,1,min(left(array)[2],right(array)[2]),max(left(array)[3],right(array)[3])
	andAssist.list_items = left_in.list_items + right_in.list_items
	andAssist.list_items.sort()
	andAssist.multiplier = (left_in.multiplier + right_in.multiplier)*andval
	if left_in.duplicates or right_in.duplicates:
		andAssist.duplicates = True
	else:
		andAssist.duplicates = searchDuplicate(left_in.list_items,right_in.list_items)
	# print(andAssist.list_items,andAssist.duplicates)
	return andAssist

def getThen(left_in,right_in):
	def thenAssist(array,left=left_in,right=right_in):
		# print(left(array)[1] , right(array)[1])
		if ((left(array)[3]<right(array)[2])*(left(array)[1] + right(array)[1] > 0)) and not thenAssist.duplicates:
		# if left(array)[1] + right(array)[1] <= 0:
			return array,1,left(array)[2],right(array)[3]
		else:
			return array,-1,left(array)[2],right(array)[3]
	thenAssist.list_items = left_in.list_items + right_in.list_items
	thenAssist.list_items.sort()
	thenAssist.multiplier = (left_in.multiplier + right_in.multiplier)*thenval
	if left_in.duplicates or right_in.duplicates:
		thenAssist.duplicates = True
	else:
		thenAssist.duplicates = searchDuplicate(left_in.list_items,right_in.list_items)
	return thenAssist

def getWhile(left_in,right_in):
	def whileAssist(array,left=left_in,right=right_in):
		# print(left(array)[1] , right(array)[1])
		if (left(array)[3]>=right(array)[2] - 1) and (left(array)[2] <= right(array)[3] +1) and not whileAssist.duplicates:
		# if left(array)[1] + right(array)[1] <= 0:
			return array,1*left(array)[1]*right(array)[1],left(array)[2],right(array)[3]
		else:
			return array,-1,left(array)[2],right(array)[3]

	whileAssist.list_items = left_in.list_items + right_in.list_items
	whileAssist.list_items.sort()
	whileAssist.multiplier = (left_in.multiplier + right_in.multiplier)*whileval
	if left_in.duplicates or right_in.duplicates:
		whileAssist.duplicates = True
	else:
		whileAssist.duplicates = searchDuplicate(left_in.list_items,right_in.list_items)
	return whileAssist

def taskFitness(gene,task_strings,bad_task_strings):
	global rule_dict,funct_dict,last_rule_number, CORRECT, NONZERO, REPEATS
	count = 0
	count2,count3 = 0,0
	check_save_rule = False

	# print("------ Starting fitness")
	# tic = time.time()
	#get string representation of the gene
	generep = str(gene[0])+gene[1]+str(gene[2])
	#test valid rules on left and right
	if (gene[0] not in rule_dict) or (gene[2] not in rule_dict):
		raise ValueError("Either %d or %d is not valid rule number."%(gene[0],gene[2]))
	#get function of rule
	if generep in funct_dict:
		# print("-------- Getting function that already exists")
		# tic2 = time.time()
		# func = funct_dict[generep][0]
		# calc score
		CORRECT = funct_dict[generep][3]
		NONZERO = funct_dict[generep][4]
		REPEATS = funct_dict[generep][5]
		return funct_dict[generep][2]
		# toc2 = time.time()
		# print('-------- Elapsed: %s' % (tic2-toc2))
	elif generep in bad_funct_dict:
		return bad_funct_dict[generep][2]
	else:
		if gene[1] == "A":
			# print("-------- Generating AND function")
			# tic2 = time.time()
			func = getAnd(funct_dict[rule_dict[gene[0]][0]][0],funct_dict[rule_dict[gene[2]][0]][0])
			# toc2 = time.time()
			# print('-------- Elapsed: %s' % (tic2-toc2))
		elif gene[1] == "O":
			# print("-------- Generating OR function")
			# tic2 = time.time()
			func = getOr(funct_dict[rule_dict[gene[0]][0]][0],funct_dict[rule_dict[gene[2]][0]][0])
			# toc2 = time.time()
			# print('-------- Elapsed: %s' % (tic2-toc2))
		elif gene[1] == "T":
			# print("-------- Generating THEN function")
			# tic2 = time.time()
			func = getThen(funct_dict[rule_dict[gene[0]][0]][0],funct_dict[rule_dict[gene[2]][0]][0])
			# toc2 = time.time()
			# print('-------- Elapsed: %s' % (tic2-toc2))
		elif gene[1] == "W":
			# print("-------- Generating WHILE function")
			# tic2 = time.time()
			func = getWhile(funct_dict[rule_dict[gene[0]][0]][0],funct_dict[rule_dict[gene[2]][0]][0])
			# toc2 = time.time()
			# print('-------- Elapsed: %s' % (tic2-toc2))
		else:
			raise ValueError("Operator not recognized. Candidates are A,O,T,W. Recieved " + gene[1])
		# #save new function to list of rules
		# last_rule_number = last_rule_number + 1
		# # print("     Gen num is:", max(rule_dict[gene[0]][1],rule_dict[gene[2]][1]) + 1)
		# # print('       left = ', rule_dict[gene[0]][1]+1, 'right', rule_dict[gene[2]][1]+1)
		# rule_dict[last_rule_number] = (generep,max(rule_dict[gene[0]][1],rule_dict[gene[2]][1]) + 1,gene[0],gene[1],gene[2])
		# funct_dict[generep] = (func,last_rule_number)
		check_save_rule = True

	# # JB: If left and right are same number, rule fails so return worst possible score? ->fixes AND
	# if gene[0] == gene[2]:
	# 	count = 0 #all considered wrong
	# 	return (2*count - len(task_strings))*(rule_dict[last_rule_number][1])
	#get score
	#  JB: POSITIVE STRINGS
	for t_str in task_strings:
		# print("-------- Evaluating good example")
		# tic2 = time.time()
		input_inds = get_inds(t_str)
		if func(input_inds)[1] == 1:
			count = count + 1
			# count2= count2 + 1
		# toc2 = time.time()
		# print('-------- Elapsed: %s' % (tic2-toc2))

	#  JB: NEGATIVE STRINGS
	for t_str in bad_task_strings:
		# print("-------- Evaluating bad example")
		# tic2 = time.time()
		input_inds = get_inds(t_str)
		if func(input_inds)[1] == -1:
			# count = count + 1
			count3 = count3 + 1
		# toc2 = time.time()
		# print('-------- Elapsed: %s' % (tic2-toc2))


	# # JB: number of examples correct*genNum - incorrect*genNum
	# #      where correct = count
	# #			 incorrect = len(strings) - count
	# # => (count)*genNum - (len(strings) - count)*genNum
	# # 		= (count - len(strings + count))*genNum
	# #		= (2*count - len(strings))*genNum
	# # return (2*count - len(task_strings))*(rule_dict[last_rule_number][1])#multiply by gen count to emphasize more complex rules
	# complexityScore = calcComplexityScore(gene)
	# correctnessScore = (2*count-len(task_strings))
	# # genNum =  (rule_dict[last_rule_number][1])
	# score = complexityScore*genNum + correctnessScore*genNum
	# print("   Fitness Calc:")
	# print("      %d * %d + %d * %d = %d" % (complexityScore, genNum, correctnessScore, genNum, score))
	# return score

	# get score components
	# print("-------- Calculating scores")
	# tic2 = time.time()
	correct = (2*count-(len(task_strings) + len(bad_task_strings)))
	(nonzero, repeats, constraints) = calcComplexityScore(gene)

	# set weights for exist criteria
	if nonzero == base_rule_length:
		W_exist = W_EXIST
	else:
		W_exist = -W_EXIST

	# calc score
	CORRECT = correct
	NONZERO = nonzero
	REPEATS = repeats

	# print("      correct: %.2f   nonzero: %.2f   repeats: %.2f" % (CORRECT, NONZERO, REPEATS))


	# TEMPORARY TESTING SCORES FOR CONSTRAINT TYPES:
	w_then = 0 #W_CORR/2 # for now make it half as important as being correct?
	then_score = constraints['T']*w_then

	# print(constraints)
	# score = 10*constraints['W'] + 1*constraints["O"] + 100*constraints['T'] + constraints['A']
	# ANDREW: adopted a multiplier style score to help push for higher level thans instead of just a than count.
	#          count = # of good string   and   count3 = # of bad strings
	score = max(count*func.multiplier - count3*func.multiplier,0)#/(max(rule_dict[gene[0]][1],rule_dict[gene[2]][1])+1)

	# score = W_CORR*correct + W_exist*nonzero - W_REP*repeats + then_score
	# score = W_CORR*correct + W_EXIST*nonzero - (base_rule_length - nonzero)
	# score = count2 + count3 - ((len(task_strings) - count2) + (len(bad_task_strings) - count3))
	# print("w1*corr + w2*nonzero - w3*repeats = score")
	# print("%.2f*%.2f + %.2f*%.2f + %.2f*%.2f = %.2f" % (W_CORR, correct, W_exist, nonzero, W_REP, repeats, score))

	# check if should save rule: if rule has score > 1/2 theoretical max
	# if( check_save_rule == True and score > SAVE_THRESH ):
	if score > max(rule_dict[gene[0]][1],rule_dict[gene[2]][1])*SAVE_THRESH:
		# print("---------- Saving rule")
		# tic3 = time.time()
		#save new function to list of rules
		last_rule_number = last_rule_number + 1
		# print("     Gen num is:", max(rule_dict[gene[0]][1],rule_dict[gene[2]][1]) + 1)
		# print('       left = ', rule_dict[gene[0]][1]+1, 'right', rule_dict[gene[2]][1]+1)
		rule_dict[last_rule_number] = (generep,max(rule_dict[gene[0]][1],rule_dict[gene[2]][1]) + 1,gene[0],gene[1],gene[2])
		funct_dict[generep] = (func,last_rule_number,score,correct,nonzero,repeats)
		# toc3 = time.time()
		# print('---------- Elapsed: %s' % (tic3-toc3))
		# print("---------- Saving child data")
		# tic3 = time.time()
		storeChildData(gene,last_rule_number)
		# toc3 = time.time()
		# print('---------- Elapsed: %s' % (tic3-toc3))

		# # TEMPORARY THINGSSSSSS:
		# print(len(constraints_dict[last_rule_number]))
		# print(constraints_dict[last_rule_number])
		# if len(constraints_dict[last_rule_number]) <= 1 : #assuming at least 2 constraints in system!!
		# 	score = -1
	else:
		bad_funct_dict[generep] = (func,-1,score)
	# return score
	# toc2 = time.time()
	# print('-------- Elapsed for calc scores: %s' % (tic2-toc2))
	# toc = time.time()
	# print('------ Elapsed for total fitness function: %s' % (tic-toc))
	# print("------Returning scores")

	return score

# NOTE: MUCH REPETITION, SUCH BAD -> should save off dict of children or something instead if this works
# JB: iterate through dictionary to count number of times each base exists in rule
#       want to rate high for each number but discount for repeated numbers
#          rate = NumsExists - totalRepeats?
def calcComplexityScore(gene):
	score = 0

	# get the counts array and the constraint counts for the rules
	counts = np.zeros(base_rule_length)
	constraints = {'T': 0, 'A': 0, 'O': 0, 'W': 0}
	constraints[gene[1]] = constraints[gene[1]] + 1
	# getCounts(gene, counts, constraints)
	getChildCounts(gene[0], counts) # counts left
	getChildCounts(gene[2], counts) # counts right
	# print("=====counts array====",counts)
	getConstraintCounts(gene[0],constraints) #left subtree
	getConstraintCounts(gene[2],constraints) #right subtree

	# get how many numbers exist
	nonzero = np.count_nonzero(counts)

	# get how many repeats
	for i in range(base_rule_length):
		if counts[i] > 0:
			counts[i] = counts[i] - 1
	# print(counts)
	repeats = np.sum(counts)

	# return score
	# score = nonzero - repeats
	# return score
	return (nonzero, repeats, constraints)

def getCounts(gene, counts, constraints):

	# rule = gene
	left = gene[0]
	right = gene[2]

	# CHECK LEFT
	if left >=0 and left < base_rule_length:
		# add left number
		# print("HIT THE LEFT BASEEEEE")
		counts[left] = counts[left] + 1
		#  save off constraint
		getConstraints(gene, constraints)
	else:
		left_rule = rule_dict[gene[0]]
		left_gene = (left_rule[2],left_rule[3],left_rule[4],)
		getCounts(left_gene,counts,constraints)

	# CHECK RIGHT
	if right >=0 and right < base_rule_length:
		# add right number
		# print("HIT THE RIGHTTTTTTTTTTT BASEEEEE")
		counts[right] = counts[right] + 1
		#  save off constraint
		# getConstraints(gene, constraints)
	else:
		right_rule = rule_dict[gene[2]]
		right_gene = (right_rule[2],right_rule[3],right_rule[4],)
		getCounts(right_gene,counts,constraints)

def getConstraints(gene,constraints):
	if gene[1] == 'T':
		constraints['T'] = constraints['T'] + 1
	if gene[1] == 'A':
		constraints['A'] = constraints['A'] + 1
	if gene[1] == 'O':
		constraints['O'] = constraints['O'] + 1
	if gene[1] == 'W':
		constraints['W'] = constraints['W'] + 1

def getChildCounts(rule_number, counts):
	# iterate through the child_dict and count the number of times it repeats
	children = child_dict[rule_number]
	for child in children:
		counts[child] = counts[child] + 1

def getConstraintCounts(rule_number, counts):
	# iterate through the child_dict and count the number of times it repeats
	constraints = constraints_dict[rule_number]
	for constraint in constraints:
		counts[constraint] = counts[constraint] + 1

#  just stores array of all numbers that are children, not base level nodes!!!
# def storeChildData(gene,last_rule_number):
# 	global child_dict

# 	#  for latest rule, update the child data from it's child rules?
# 	# rule = gene
# 	left = gene[0]
# 	right = gene[2]
# 	#get string representation of the gene
# 	generep = str(gene[0])+gene[1]+str(gene[2])

# 	print("LEFT %s RIGHT %s RULE %s" % (left, right, last_rule_number))
# 	# for the last rule need to iterate through the rule and save off stuffs????

# 	# ------ store left -----
# 	if last_rule_number >= 0 and last_rule_number < base_rule_length:
# 		# if left is original number, store that number
# 		child_dict[last_rule_number] = child_dict[last_rule_number] + [left]
# 	else:
# 		# store the list from left plus the left rule
# 		child_rule = child_dict[gene[0]]
# 		print("child: %s" % child_rule)
# 		child_dict[last_rule_number] = child_rule + [gene[0]]

# 	# ------ store right -----
# 	if last_rule_number >= 0 and last_rule_number < base_rule_length:
# 		# if right is original number, store that number
# 		child_dict[last_rule_number] = child_dict[last_rule_number] + [right]
# 	else:
# 		# store the list from right plus the right rule (and list from the left that is already stored)
# 		child_rule = child_dict[gene[2]]
# 		print("child: %s" % child_rule)
# 		child_dict[last_rule_number] = child_dict[last_rule_number] + child_rule + [gene[2]]

# stores children as array of base level nodes
def storeChildData(gene,last_rule_number):
	global child_dict
	global constraints_dict

	#  for latest rule, update the child data from it's child rules?
	# rule = gene
	left = gene[0]
	right = gene[2]
	#get string representation of the gene
	generep = str(gene[0])+gene[1]+str(gene[2])
	dirty_bit = False

	# print("LEFT %s RIGHT %s RULE %s" % (left, right, last_rule_number))
	# for the last rule need to iterate through the rule and save off stuffs????

	# ------ store constraint ------
	if last_rule_number in constraints_dict.keys():
		constraints_dict[last_rule_number] = constraints_dict[last_rule_number] + [gene[1]]
	else:
		constraints_dict[last_rule_number] = [gene[1]]

	# ------ store left -----
	if left >=0 and left < base_rule_length:
		# if left is original number, store that number
		if last_rule_number in child_dict.keys():
			child_dict[last_rule_number] = child_dict[last_rule_number] + [left]
			# dirty_bit = True
			# constraints_dict[last_rule_number] = constraints_dict[last_rule_number] + [gene[1]]
		else:
			child_dict[last_rule_number] = [left]
			# dirty_bit = True
			# constraints_dict[last_rule_number] = [gene[1]]
	else:
		# store the list from left plus the left rule
		left_rule = rule_dict[gene[0]]
		left_gene = (left_rule[2],left_rule[3],left_rule[4],)
		storeChildData(left_gene, last_rule_number)

	# ------ store right -----
	if right >=0 and right < base_rule_length:
		# if right is original number, store that number
		if last_rule_number in child_dict.keys():
			child_dict[last_rule_number] = child_dict[last_rule_number] + [right]
			# if dirty_bit == False:
				# constraints_dict[last_rule_number] = constraints_dict[last_rule_number] + [gene[1]]
		else:
			child_dict[last_rule_number] = [right]
			# if dirty_bit == False:
				# constraints_dict[last_rule_number] = [gene[1]]
	else:
		# store the list from right plus the right rule (and list from the left that is already stored)
		right_rule = rule_dict[gene[2]]
		right_gene = (right_rule[2],right_rule[3],right_rule[4],)
		storeChildData(right_gene, last_rule_number)


def initRules(baseObjects):
	global rule_dict,funct_dict,last_rule_number,base_rule_length
	base_rule_length = baseObjects
	for a in range(base_rule_length):
		rule_dict[a] = (str(a),0,a,"B",a)#gen 0, base rule a, operator base, base rule a: last three used for printing.
		last_rule_number = a
		funct_dict[str(a)] = (getBase(a),a,-1)#rule dict key
		child_dict[a] = []
		constraints_dict[a] = []

def printRule(rulenumber, filename, base=True):

	if base:
		filename = open(filename,"a")
	if rule_dict[rulenumber][3] == "B":
		filename.write("(PLACE "+str(rulenumber)+")")
	else:
		filename.write("(")
		filename.write(rule_dict[rulenumber][3])
		printRule(rule_dict[rulenumber][2],filename,False)
		
		printRule(rule_dict[rulenumber][4],filename,False)
		filename.write(")")
	if base:
		filename.write("\n\n")
		filename.close()
def printRulestr(rulenumber, base=True):
	print(rulenumber)
	print("\n")

	if rule_dict[rulenumber][3] == "B":
                
                string.append("(PLACE "+object_list[rulenumber]+")")

		
	else:
		string.append("(")
		string.append(rule_dict[rulenumber][3])
		printRulestr(rule_dict[rulenumber][2],False)
		
		printRulestr(rule_dict[rulenumber][4],False)
		string.append(")")
	if base:
		string.append("\n\n")

def printChildDict():
	for key in child_dict:
		print('Key: %s Dict: %s' % (key, child_dict[key]) )

def printConstraintsDict():
	for key in constraints_dict:
		print('Key: %s Dict: %s' % (key, constraints_dict[key]) )

def printRuleDict(filename):
    f = open(filename,"w")
    for i in range(last_rule_number+1):
        f.write(str(i)+": ")
        printRule(i, f, base=False)
        f.write("\n\n")
    f.close()
def getPrintRuleStr(rulenumber):
     global string

     printRulestr(rulenumber,base=False)
     return string


if __name__ == '__main__':
	# JB: these will be the demonstrations given
	input_array = [0,2,3,1,4,5]
	pass_input_array = get_inds(input_array)
	print(pass_input_array)

	# for a in range(base_rule_length):
	# 	rule_dict[a] = (str(a),0)#gen 0
	# 	last_rule_number = a
	# 	funct_dict[str(a)] = (getBase(a),a)#rule dict key

	#JB: This is generating the initial rules for the individual objects!!!
	initRules(base_rule_length)
	# for a in range(base_rule_length):
	# 	print(funct_dict[str(a)][0](pass_input_array))

	# JB: Generate rule for AND between two objects
	andTest1 = getAnd(funct_dict[str(1)][0],funct_dict[str(2)][0])
	andTest2 = getAnd(funct_dict[str(1)][0],funct_dict[str(6)][0])

	# JB: Test if the rule passed or not using demonstration
	print("and1",andTest1(pass_input_array))
	print("and2",andTest2(pass_input_array))

	orTest1 = getOr(funct_dict[str(3)][0],funct_dict[str(4)][0])
	orTest2 = getOr(funct_dict[str(3)][0],funct_dict[str(6)][0])

	print("or1",orTest1(pass_input_array))
	print("or2",orTest2(pass_input_array))

	thenTest1 = getThen(orTest1,funct_dict[str(5)][0])
	thenTest2 = getThen(orTest2,funct_dict[str(1)][0])
	thenTest3 = getThen(funct_dict[str(6)][0],orTest2)

	print("then1",thenTest1(pass_input_array))
	print("then2",thenTest2(pass_input_array))
	print("then3",thenTest3(pass_input_array))

	whileTest1 = getWhile(andTest1,funct_dict[str(5)][0])
	whileTest2 = getWhile(orTest1,funct_dict[str(3)][0])

	print("while1",whileTest1(pass_input_array))
	print("while2",whileTest2(pass_input_array))

	# JB: Test fitness function
	print(taskFitness((1,'A',5),[input_array]))
