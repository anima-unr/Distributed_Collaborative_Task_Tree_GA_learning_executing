
# basic storage and config params
base_rule_length = 6 # CHANGE THIS TO FIT LENGTH OF TASK STRING!!! -- TEAMTIME
# base_rule_length = 5 # OR example
pop_size =  1000
w_corr = 5
w_exist = 2
w_rep = 1
num_iters = 20

# task_strings = [[0,2,3,1,4,5],[0,2,3,1,5,6]] # RANDOM
# task_strings = [[0,1,2,3,4],[1,0,2,3,4],[0,1,2,4,3],[1,0,2,4,3]] #AND for 01 and 34, then otherwise

# THEN ROOT EXAMPLE
# task_strings = [[0,1,3,4,5,6],[0,1,2,4,5,6],[0,1,3,4,6,5],[0,1,2,4,6,5]] #THEN inbetween tea&sandwich with THEN BREAD BREAD M OR L because I messed up
# bad_task_strings = [[6,5,4,3,2,1,0],[5,4,6,2,3,1,0]]
# # bad_task_strings = [[6,5,4,3,2,1,0],[5,4,6,2,3,1,0],[3,2,1,0,6,5,4],[2,3,1,0,5,4,6]]#,[1,2,3,5,6,0,4],[1,2,5,3,6,0,4],[3,6,5,2,4,1,0]] #tea&sandwich bad

# task_strings = [[0,3,1,4,5,6],[0,2,1,4,5,6],[0,3,1,4,6,5],[0,2,1,4,6,5]] #THEN inbetween tea&sandwich 
# bad_task_strings = [[6,5,4,3,2,1,0],[5,4,6,2,3,1,0]]
#task_strings = [[0,3,1,4,5,6],[0,2,1,4,5,6],[0,3,1,4,6,5],[0,2,1,4,6,5]] #THEN inbetween tea&sandwich 
#bad_task_strings = [[6,5,4,3,2,1,0],[5,4,6,2,3,1,0]]

# task_strings = [[0, 1, 2, 3, 5, 4], [1, 0, 3, 2, 4, 5], [0,1,2,3,4,5]]
# bad_task_strings = [[4,3,5,2,0,1], [2,3,4,5,0,1],[1,2,3,0,4,5]

# task_strings = [[0, 1, 2, 3, 5, 4], [1, 0, 3, 2, 4, 5], [0,1,3,2,4,5]]
task_strings = [[0, 1, 2, 3, 4], [0, 1, 3, 2, 5], [1, 0, 3, 2, 4]]
bad_task_strings = [[3,5,2,0,1], [2,3,4,0,1]]


# AND ROOT EXAMPLE
#task_strings = [[0,1,3,4,5,6],[0,1,2,4,5,6],[0,1,3,4,6,5],[0,1,2,4,6,5],[4,5,6,0,1,3],[4,5,6,0,1,2],[4,6,5,0,1,3],[4,6,5,0,1,2]] #AND at top of tea&sandwich with THEN BREAD BREAD M OR L because I messed up
#bad_task_strings = [[1,0],[5,4,6],[6,4,5],[3,2,1,0],[2,3,1,0]]


#OR ROOT EXAMPLE
# # task_strings = [[0,1,2,3],[0,1,2,3],[4,6,5],[4,5,6]] #OR at top of tea&sandwich with THEN BREAD BREAD M OR L because I messed up
# # bad_task_strings = [[3,2,1,0,5,4,6],[3,2,1,0,6,4,5]]
# task_strings = [[0,1],[2],[1,0]] #OR at top of tea&sandwich with THEN BREAD BREAD M OR L because I messed up
# bad_task_strings = [[2,1,0]]

# task_strings = [[0,1],[2,3],[1,0]] #OR at top of tea&sandwich with THEN BREAD BREAD M OR L because I messed up
# bad_task_strings = [[2,1,0,3]]

# task_strings = [[0,1],[1,0],[2,3,4],[3,2,4]] #OR at top of tea&sandwich with THEN BREAD BREAD M OR L because I messed up
# bad_task_strings = [[2,1,4,0,3],[2,4,3,1,0]]


# WORKING:
# task_strings = [[0,1,2],[0,2,1]] # TEA
# task_strings = [[0,1,3],[0,1,2]] #OR

andval = 1 #weight to factor in "priority" of AND
orval = 1  #weight to factor in "priority" of OR
thenval = 4 #weight to factor in "priority" of THEN
whileval = 4 #weight to factor in "priority" of WHILE

bestholdover = .7 # keep 95% of each pop every iter

