#!/usr/bin/env python3

import numpy as np
import time
from deap import creator, base, tools, algorithms
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
from sound_play.libsoundplay import SoundClient

import config
from parse_task_tree import *
from reorder import *
from publishing_command import *
from publishing_command_display import *
from GoogleSpeechToText import *

base_rule_length = config.base_rule_length
pop_size = config.pop_size
task_strings = config.task_strings
bad_task_strings = config.bad_task_strings
W_CORR = config.w_corr
W_EXIST = config.w_exist
W_REP = config.w_rep
str_tree=[]

import task_ops
import rospy
from std_msgs.msg import String

pub_str = rospy.Publisher('/listener', String, queue_size=100)

object_list = ["blue_leg","green_leg","yellow_bar","pink_bar","orange_top","purple_top"]

# sound_handle=SoundClient()

# MAX_FITNESS = W_CORR*base_rule_length + W_EXIST*base_rule_length
MAX_FITNESS = W_CORR*(len(config.task_strings)+len(config.bad_task_strings)) + W_EXIST*base_rule_length
NUM_ITERS = config.num_iters
NUM_CONSTRAINTS = 3 #for now only gen THEN AND OR and do NOT generate WHILE -> for WHILE set to 4

#=============================================================

#=============================================================
#TODOs:
#---------------------------------------------------------
# 1. how to keep track of repeats of individuals or rules?!
# 2. using numpy make sure to read: https://deap.readthedocs.io/en/master/tutorials/advanced/numpy.html


# ===============================================================================
# ===============================================================================

# #--------------------------------------
# # generate an individual string
# #--------------------------------------
# def genIndividualString():

#     # generate form <num><constraint><num>
#     nums = np.random.randint(task_ops.get_last_rule_number(), size=2) #should be length of dict or last rule num when actually using!
#     # nums = np.random.randint(base_rule_length, size=2) # for testing, make it size of inital objects

#     while nums[0] == nums[1]:
#         # print("\tgenerated repeat:", nums[0], nums[1])
#         nums[1] = np.random.randint(task_ops.get_last_rule_number(), size=1)

#     constraint_type = np.random.randint(4, size=1)

#     if constraint_type == 0:
#         constraint = 'T'
#     elif constraint_type == 1:
#         constraint = 'A'
#     elif constraint_type == 2:
#         constraint = 'O'
#     elif constraint_type == 3:
#         constraint = 'W'

#     rule = (int(nums[0]), constraint, int(nums[1]))
#     # print("Generated rule:", rule)
#     return rule

# #--------------------------------------
# # generate an population of strings
# #--------------------------------------
# def genPopulationStrings(num_strings):

#   pop = []
#   for i in range(num_strings):
#       pop.append(genIndividualString())
#   # print("pop is:", pop)
#   return pop

# #--------------------------------------
# # generate rule from string
# #--------------------------------------
# DONT NEED THIS SINCE FITNESS FUNCTION GENERATES THE FUNCTIONS!
# def genRuleFromString(rule_string):

#   # call approporate task_op function based on constraint
#   rule = ()
#   print("rule string is:", rule_string)
#   if rule_string[1] == 'T':
#       rule = task_ops.getThen(funct_dict[rule_string[0]][0],funct_dict[rule_string[2]][0])
#   elif rule_string[1] == 'A':
#       rule = task_ops.getAnd(funct_dict[rule_string[0]][0],funct_dict[rule_string[2]][0])
#   elif rule_string[1] == 'O':
#       rule = task_ops.getOr(funct_dict[rule_string[0]][0],funct_dict[rule_string[2]][0])
#   elif rule_string[1] == 'W':
#       rule = task_ops.getWhile(funct_dict[rule_string[0]][0],funct_dict[rule_string[2]][0])

#   # rule_dict[???] = rule

#   print("rule is: ", rule)
#   return rule

# #--------------------------------------
# # generate rule from population
# #--------------------------------------
#  DONT NEED THIS SINCE  FITNESS ADDS GENERATED RULES
# def genRulesFromPopulation(pop):

#   rules = [] # shouldn't need to keep since should store in rules dict!
#   for i in range(len(pop)):
#       # print i
#       rules.append(genRuleFromString(pop[i]))
#   print("rules", rules)
#   return rules


#--------------------------------------
# evaluation function created in task_ops.py
#--------------------------------------
def evalGA(individual):
    # return score from fitness function
    # print("individual", individual)
    return (task_ops.taskFitness(individual, task_strings, bad_task_strings),)

#--------------------------------------
# custom mutation function
#--------------------------------------
def mutString(individual):

    mutated = individual

    # generate random position for mutation
    pos = np.random.randint(4, size=1)

    # NOTE: RIGHT NOW CAN HAVE SAME # ON LEFT AND RIGHT, FIX LATER!!!!!!
    # if position is 0 or 2 generate new number within bounds of rule_dict
    if pos == 0:
         mut = np.random.randint(task_ops.get_last_rule_number(), size=1)
         while mut == individual[2] or mut == individual[0]:
            mut = np.random.randint(task_ops.get_last_rule_number(), size=1)
         mutated[0] = int(mut)

    elif pos == 2:
         mut = np.random.randint(task_ops.get_last_rule_number(), size=1)
         while mut == individual[0] or mut == individual[2]:
            mut = np.random.randint(task_ops.get_last_rule_number(), size=1)
         mutated[2] = int(mut)

    #  else if position is 1 generate new constraint
    #  have 50% chance of mutating the constraint , (25% mutation of left number and right number each)
    elif pos == 1 or pos == 3:
        constraint_type = np.random.randint(NUM_CONSTRAINTS+1, size=1)
        if constraint_type == 0 or constraint_type == 1: #generate THEN more often
            mutated[1] = 'T'
        elif constraint_type == 2:
            mutated[1] = 'A'
        elif constraint_type == 3:
            mutated[1] = 'O'
        elif constraint_type == 4:
            mutated[1] = 'W'

    # return mutated individual
    return mutated

def genConstraint():
    constraint_type = np.random.randint(NUM_CONSTRAINTS+1, size=1)
    if constraint_type == 0 or constraint_type == 1:
        constraint = 'T'
    elif constraint_type == 2:
        constraint = 'A'
    elif constraint_type == 3:
        constraint = 'O'
    elif constraint_type == 4:
        constraint = 'W'
    return constraint

def genNum():
    return int(np.random.randint(task_ops.get_last_rule_number(), size=1))

def genNum2():
    return int(np.random.randint(base_rule_length, size=1))

#=============================================================
#=============================================================


#--------------------------------------
# fitness function
#--------------------------------------
creator.create("FitnessMax", base.Fitness, weights=(1.0,))

#--------------------------------------
# definining individuals?
#--------------------------------------
creator.create("Individual", list, fitness=creator.FitnessMax) #NOTE: list or tuple?!?!

toolbox = base.Toolbox()
toolbox.register("attr_constr", genConstraint)
toolbox.register("attr_num", genNum)
toolbox.register("attr_num2", genNum2)
toolbox.register("individual", tools.initCycle, creator.Individual,
                 (toolbox.attr_num, toolbox.attr_constr, toolbox.attr_num,), n=1)
toolbox.register("individual2", tools.initCycle, creator.Individual,
                 (toolbox.attr_num2, toolbox.attr_constr, toolbox.attr_num2,), n=1)

#--------------------------------------
# defining population
#--------------------------------------
toolbox.register("population", tools.initRepeat, list, toolbox.individual)
toolbox.register("population2", tools.initRepeat, list, toolbox.individual2)

#--------------------------------------
# defining evaluation
#--------------------------------------
toolbox.register("evaluate", evalGA)

#--------------------------------------
# mutation function?
#--------------------------------------
toolbox.register("mutate", mutString)

#--------------------------------------
# mating function?
#--------------------------------------
toolbox.register("mate", tools.cxOnePoint) # 1 point crossover on tuple????? or have to be list??
# toolbox.register("select", tools.selTournament, tournsize=3)
# toolbox.register("select",tools.selTournament,tournsize=5)
toolbox.register("select",tools.selRoulette) # use roulette selection method to keep

#--------------------------------------
# GA algorithm?
#--------------------------------------
# simple one: pop, toolbox, prob mate, prob mutate, num iters
def runGA( ):
    bestRule = 0
    bestFitness = -100000
    # # TESTING INDIVIDUAL FIRST
    # # evaluate individual
    # ind1 = toolbox.individual()
    # print(ind1)
    # print(ind1.fitness.valid)
    # ind1.fitness.values = evalGA(ind1)
    # print(ind1.fitness.valid)
    # print(ind1.fitness)

    # create the population
    pop = toolbox.population(n=pop_size)
    # print(pop)

    # Evaluate the entire population
    print("**EVALUATING INITIAL POPULATION:**")
    fitnesses = list(map(toolbox.evaluate, pop))
    for ind, fit in zip(pop, fitnesses):
        ind.fitness.values = fit
    #     print("    ind: %s   fit: %.2f " % (ind, fit[0]))

    # CXPB  is the probability with which two individuals are crossed
    # MUTPB is the probability for mutating an individual
    CXPB, MUTPB = .8, 0.4

    # Extracting all the fitnesses of
    fits = [ind.fitness.values[0] for ind in pop]

    # Variable keeping track of the number of generations
    g = 0

    # Begin the evolution
    # while max(fits) < MAX_FITNESS and g < NUM_ITERS:
    while g < NUM_ITERS:
        # A new generation
        g = g + 1
        print("\n-- Generation %i --" % g)


        # Select the next generation individuals -> take top 50%
        # offspring = toolbox.select(pop, len(pop)/2)
        offspring = toolbox.select(pop, np.floor(len(pop)*config.bestholdover).astype(int))
        # Clone the selected individuals
        offspring = list(map(toolbox.clone, offspring))

        # Generate a random %50 new offspring
        pop2 = toolbox.population(n=np.ceil(pop_size*(1 - config.bestholdover/2)).astype(int))
        pop3 = toolbox.population2(n=np.ceil(pop_size*(1 - config.bestholdover/2)).astype(int))

        # Add to offspring
        offspring2 = list(map(toolbox.clone, pop2)) + list(map(toolbox.clone,pop3))
        offspring = offspring + pop2 + pop3
        # print("   Starting pop: %s", offspring)

        # Apply crossover and mutation on the offspring
        # WHAT THIS DOES: get odd elements for child 1 and even elements for child2 to make pairs
        for child1, child2 in zip(offspring[::2], offspring[1::2]):
            if np.random.random() < CXPB:
                toolbox.mate(child1, child2)
                del child1.fitness.values
                del child2.fitness.values
                # print("  Crossover: off1, off2, child1, child2")
                # print("     %s, %s, %s, %s " % (offspring[::2], offspring[1::2], child1, child2))

        for mutant in offspring:
            if np.random.random() < MUTPB:
                toolbox.mutate(mutant)
                del mutant.fitness.values
                # print("  Mutation: ")
                # print("     %s" % mutant)

        # Evaluate the individuals with an invalid fitness
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses = map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit
        #     print("  Evaluating new offstring not already in population")
        #     print("    ind: %s   fit: %.2f " % (ind, fit[0]))

        # replace old population
        pop[:] = offspring
        # print("   Ending pop: %s", offspring)

        # Gather all the fitnesses in one list and print the stats
        fits = [ind.fitness.values[0] for ind in pop]

        length = len(pop)
        mean = sum(fits) / length
        sum2 = sum(x*x for x in fits)
        std = abs(sum2 / length - mean**2)**0.5

        print("  Min %s" % min(fits))
        print("  Max %s" % max(fits))
        if max(fits) >= bestFitness:
            bestRule = pop[fits.index(max(fits))]
            bestFitness = max(fits)
        print("  Avg %s" % mean)
        print("  Std %s" % std)

        print("  Current Population best rule: %s" % pop[fits.index(max(fits))])
        print("    Fitness: %.2f" % max(fits))
        # print("       MAXIMUM POSSIBLE FITNESS FOR EXAMPLE IS: %d" % MAX_FITNESS)

        # Get corresponding rule if exists in dictionary
        print("  Overall best rule: %s" % bestRule)
        print("    Fitness: %.2f" % bestFitness)
        # generep = str(pop[fits.index(max(fits))][0])+pop[fits.index(max(fits))][1]+str(pop[fits.index(max(fits))][2])
        generep = str(bestRule[0])+bestRule[1]+str(bestRule[2])
        if generep in task_ops.get_funct_dict():
            rule_number = task_ops.get_funct_dict()[generep][1]
            print("    Corresponding rule in dict: %d" % rule_number)
            best = (pop[fits.index(max(fits))], rule_number)
        else:
            print("     Rule has too low of fitness, not saved in dict, no best found yet.")
            best = -1


    return best

# ===============================================================================
# ===============================================================================

def main_algo():

    global str_tree


        # Set up the demonstrations (just one for now)  - use config task_strings instead
    # input_array = [0,2,3,1,4,5]
    # pass_input_array = task_ops.get_inds(input_array)

    #This is generating the initial rules for the individual objects!!!
    print("== Starting program =============================================")

    tic_prog = time.time()
    task_ops.initRules(base_rule_length)
    # for a in range(base_rule_length):
        # print(funct_dict[str(a)][0](pass_input_array))

    # rule_string = genIndividualString()
    # pop = genPopulationStrings(pop_size)
    # genRuleFromString(rule_string)
    # genRulesFromPopulation(pop)


    # Now call the GA to generate pops and such for N iterations?
    print("\n-- Starting GA --------------------------------------")
    tic_GA = time.time()
    best = runGA()
    toc_GA = time.time()
    print('-- Elapsed GA: %s ---------------------------------------' % (tic_GA-toc_GA))

    # print("Rule dict:", task_ops.get_rule_dict())
    # NOTE: only gives best in currect pop not dict!!!!!!!
    # print("Best is: ", best)
    # print("Keys for dict:", task_ops.get_rule_dict().keys())

    print("\n-- Printing Rule Dict to file ------------------------")
    tic_GA = time.time()

    filename = "output.txt"
    task_ops.printRuleDict(filename)
    # task_ops.printChildDict()
    # task_ops.printConstraintsDict()

    toc_GA = time.time()
    print('-- Elapsed Print: %s ---------------------------------------' % (tic_GA-toc_GA))

    # Decode the latest rule!  - and get fitness for it!!!
    print("\n-- Decoding rules to get best rule  ------------------------")
    tic_GA = time.time()

    rules = task_ops.get_rule_dict()
    highest_fit = -1000
    highest_rule = -1
    data=(-1,-1,-1,-1)
    for rule in rules:
        if rules[rule][3] != 'B': # don't evaluate base rules
            fitness = task_ops.taskFitness(rules[rule][2:],task_strings, bad_task_strings)
            # TEMPORARY THINGSSSSSS:
            temp = task_ops.get_score_comps()
            # print(len(temp[3][rule]))
            # print(temp[3][rule])
            if len(temp[3][rule]) < int(base_rule_length/2): #Make sure its not too short!
                fitness = -1
            # print(fitness)
            if fitness > highest_fit:
                highest_fit = fitness
                highest_rule = rule
                data = task_ops.get_score_comps()
    # rule_number = task_ops.get_funct_dict()[highest_rule[0]][1]
    print("\n\nBest Rule in Dictionary is:\n" )
    print("   %d: %s with fit of %.2f" % (highest_rule, rules[highest_rule], highest_fit))
    print("      correct: %.2f   nonzero: %.2f   repeats: %.2f" % (data[0], data[1], data[2]))
    print("         constraints: %s" % data[3][highest_rule])
    print("       MAXIMUM POSSIBLE FITNESS FOR EXAMPLE IS: %d" % MAX_FITNESS)

    toc_GA = time.time()
    print('-- Elapsed Decode: %s ---------------------------------------' % (tic_GA-toc_GA))


    print("== Ending program =============================================")
    toc_prog = time.time()
    print("== Elapsed total: %s ================================================" % (tic_prog - toc_prog))
    
    str_tree=task_ops.getPrintRuleStr(highest_rule)
    print(str_tree)



    


    for i in range(len(str_tree)):
        if str_tree[i] =="T":
            str_tree[i]="THEN"
        elif str_tree[i] =="A":
            str_tree[i]="AND"
        elif str_tree[i] =="O":
            str_tree[i]="OR"
    
    str_tree = ''.join(str_tree)
    
    print(str_tree)

    pub_str.publish(str_tree)
    time.sleep(3)
    say("Please check your nearby tab and let me know if the given sequence is correct or not. Please reply with Yes or No.")
    time.sleep(3)
		

if __name__ == '__main__':
    rospy.init_node('main_algo', anonymous=True)
    # time.sleep(5)

    say("Thank you for your time. I am starting to learn and generate the task sequence. Please check the nearby tab on the table for learning update.")
    pub_str.publish("Learning ...")
    main_algo()
    while(1):

        # str_tree = "(THEN(AND(THEN(PLACE blue_leg)(PLACE yellow_bar))(THEN(PLACE green_leg)(PLACE pink_bar)))(AND(PLACE orange_top)(PLACE purple_top)))"
        print(str_tree)
        # time.sleep(20)
        pub_str.publish(str_tree)
        
        say("Please check your nearby tab and let me know if the given sequence is correct or not. Please reply with Yes or No.")
        time.sleep(3)

        # response = speech_rec()
        response = input("Is this sequence corrent? Please say Yes or No.")
        print("here",response)
        if response=="yes":
            say("Thank you. Please help me to assemble the table top with me.")
            main_parse_task_tree(str_tree)
            main_reorder()
            main_publisher()
            break
        elif response=="no":
            say("Thanks for letting me know. I am starting to learn and generate the task sequence again.")
            task_ops.string = []
            str_tree = []
            task_ops.last_rule_number = 0
            task_ops.rule_dict = {}
            task_ops.funct_dict = {}
            task_ops.bad_funct_dict = {}
            task_ops.child_dict = {}
            task_ops.constraints_dict = {}
            main_algo()
        # else:
            # say("please say again")




    

    

    # f = open(filename,"w") #quick and dirty way to clear the file for each run
    # f.close()

    # for i in range(task_ops.get_last_rule_number()+1):
    #     task_ops.printRule(i, filename, base=True)
    #     # print("index of rule to try to print: ", i)
