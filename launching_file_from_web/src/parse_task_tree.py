#!/usr/bin/env python

import sys
'''
Description: A program to parse the Task Expression Language
Output => A preorder tree traversal and a YAML file
'''

Symbol = str
outfilestream = open('outputnew.txt', 'w')

# Tokenize the input stream of characters based on the language
def tokenize(chars):
  return chars.replace('(', ' ( ').replace(')', ' ) ').split()

def read_from_tokens(tokens):
  if len(tokens) == 1:
    raise SyntaxError('Unexpected EOF while reading')
  token = tokens.pop(0)
  if '(' == token:
    L = []
    while tokens[0] != ')':
      L.append(read_from_tokens(tokens))
    tokens.pop(0)
    return L
  elif ')' == token:
    raise SyntaxError('Unexpected [)] symbol')
  else:
    return atom(token)

def atom(token):
  try: return int(token)
  except ValueError:
    try: return float(token)
    except ValueError:
      return Symbol(token)

#   Parse the input character stream and output syntax tree
def parse(program):
  print(program)
  program = "(ROOT" + program + ")"
  return read_from_tokens(tokenize(program))

class Env(dict):
  "An environment: a dict of {'var':val} pairs, with an outer Env."
  def __init__(self, parms=(), args=(), outer=None):
    self.update(zip(parms, args))
    self.outer = outer if outer else {}
  def find(self, var):
    "Find the innermost Env where var appears."
    return self if (var in self) else None

def CreateRootObject( robot_id, node_id, parent='NONE'):
  return TaskObject('ROOT', 4, robot_id, node_id, parent)

def CreateThenObject(robot_id, node_id, parent):
  return TaskObject('THEN', 0, robot_id, node_id, parent)

def CreatePlaceObject(robot_id, node_id, parent):
  return PlaceObject('PLACE', 6, robot_id, node_id, parent)

def CreateAndObject(robot_id, node_id, parent):
  return TaskObject('AND', 2, robot_id, node_id, parent)

def CreateOrObject(robot_id, node_id, parent):
  return TaskObject('OR', 1, robot_id, node_id, parent)

def CreateWhileObject(robot_id, node_id, parent):
  return TaskObject('WHILE', 8, robot_id, node_id, parent)

def CreateRootObject_baxter( robot_id, node_id, parent='NONE'):
  return TaskObject('ROOT', 4, 1, node_id, parent)

def CreateThenObject_baxter(robot_id, node_id, parent):
  return TaskObject('THEN', 0, 1, node_id, parent)

def CreatePlaceObject_baxter(robot_id, node_id, parent):
  return PlaceObject('PLACE', 6, 1, node_id, parent)

def CreateAndObject_baxter(robot_id, node_id, parent):
  return TaskObject('AND', 2, 1, node_id, parent)

def CreateOrObject_baxter(robot_id, node_id, parent):
  return TaskObject('OR', 1, 1, node_id, parent)

def CreateWhileObject_baxter(robot_id, node_id, parent):
  return TaskObject('WHILE', 8, 1, node_id, parent)

#to do: figure out why while isnt listed in the node list and all the peers for a while and its children remain as 'none'
class TaskObject(object):
  def __init__(self, name, node_type, robot_id, node_id=0, parent=''):
    self.node_type = node_type
    self.robot_id = robot_id
    self.node_id = node_id
    self.name = '%s_%d_%d_%03d' % (name, node_type, robot_id, node_id)
    self.children = ['NONE']
    self.parent = parent
    self.peers = ['NONE']

  def __call__(self, *args):
    self.children = [child.name for child in args]
    # print(self)
    # outfilestream.write("\n")
    outfilestream.write(str(self))
    return self

  def __repr__(self):
    string = (
      '%(name)s:\n'
      '  mask:\n'
      '    type: %(node_type)d\n'
      '    robot: %(robot_id)d\n'
      '    node: %(node_id)d\n'
      '  parent: %(parent)s\n'
      '  children: %(children)s\n'
      '  peers: %(peers)s\n'
      % {
        'name': self.name,
        'node_type': self.node_type,
        'robot_id': self.robot_id,
        'node_id': self.node_id,
        'parent': self.parent,
        'children': self.children,
        'peers': self.peers
      }
    )
    return string

class PlaceObject(TaskObject):
  def __init__(self, name='', node_type=0, robot_id=0, node_id=0, parent=''):
    super(PlaceObject, self).__init__(name, node_type, robot_id, node_id, parent)
    self.place_object = ''

  def __call__(self, item):
    self.place_object = item
    # print(self)
    # outfilestream.write("\n")
    outfilestream.write(str(self))
    return self

  def __repr__(self):
    parent_str = super(PlaceObject, self).__repr__()
    string = '%s  object: %s\n' % (parent_str, self.place_object)
    return string

def standard_env(robot_id):
  env = Env()
  if robot_id == 0:
      env.update({
        'THEN':    CreateThenObject,
        'PLACE':   CreatePlaceObject,
        'AND':     CreateAndObject,
        'OR':      CreateOrObject,
        'ROOT':    CreateRootObject,
        'WHILE':   CreateWhileObject,
      })
  else:
        env.update({
          'THEN':    CreateThenObject_baxter,
          'PLACE':   CreatePlaceObject_baxter,
          'AND':     CreateAndObject_baxter,
          'OR':      CreateOrObject_baxter,
          'ROOT':    CreateRootObject_baxter,
          'WHILE':   CreateWhileObject_baxter,
        })
  return env
pr2_env = standard_env(0)
baxter_env = standard_env(1)

#   "Evaluate an expression in an environment."
def eval(x, env, func_index=[0], parent="'NONE'"):
  if isinstance(x, Symbol):      # x is a string
    if env.find(x):
      value = env.find(x)[x](0, func_index[0], parent)
      func_index[0] += 1
      return value
    return x
  else: # x is a list
    proc = eval(x[0], env, parent=parent)
    args = [eval(arg, env, parent=proc.name) for arg in x[1:]]
    return proc(*args)
def main_parse_task_tree(string1):
  # string = ("(THEN(OR(PLACE one)(PLACE two))(AND(AND(PLACE three)(PLACE four))(AND(PLACE five)(PLACE six))))")
  # string2 = ("(THEN(OR(PLACE one)(PLACE two))(AND(AND(PLACE three)(PLACE four))(AND(PLACE five)(PLACE six))))")
  parse_str = parse(string1)
  parse_str2 = parse(string1)

  

  # outfilestream.write(str(parse_str))
  # outfilestream.write(str(parse_str2))
  # outfilestream.close()
  print("PARSE STR\n\n\n\n")
  # print(parse_str)
  outfilestream.write(str(parse_str))
  x = eval(parse_str, pr2_env)
  x2 = eval(parse_str2, baxter_env)
  # print("XXXXXXXXXXXXXXXXXXX")
  # print(str(x2))
  # outfilestream.write(str(x))
  # outfilestream.write(str(x2))
  outfilestream.close()

#edit strings to fit structure of tree
# if __name__ == '__main__':
#   main_parse_task_tree()

 #  string = (
 #      "(THEN "
	# "(AND"
	# 	"(PLACE apple) "
	# 	"(PLACE teddy_bear)) "
	# "(OR"
	# 	"(PLACE clock) "
	# 	"(PLACE scissors))) "
 #      )
 #  string2 = (
 #      "(THEN "
	# "(AND"
	# 	"(PLACE apple) "
	# 	"(PLACE teddy_bear)) "
	# "(OR"
	# 	"(PLACE clock) "
	# 	"(PLACE scissors))) "
  # )
  # string = ("(THEN(OR(PLACE one)(PLACE two))(AND(AND(PLACE three)(PLACE four))(AND(PLACE five)(PLACE six))))")
  # string2 = ("(THEN(OR(PLACE one)(PLACE two))(AND(AND(PLACE three)(PLACE four))(AND(PLACE five)(PLACE six))))")
  # string = (
  #  "(THEN "
  #  "(OR"
  #  "(PLACE 1) "
  #  "(PLACE 2)) "
  #  "(AND"
  #  "(AND"
  #  "(PLACE 3) "
  #  "(PLACE 4)) "
  #  "(AND"
  #  "(PLACE 5) "
  #  "(PLACE 6)))) "
  # )
  # string2 = (
  #  "(THEN "
  #  "(OR"
  #  "(PLACE 1) "
  #  "(PLACE 2)) "
  #  "(AND"
  #  "(AND"
  #  "(PLACE 3) "
  #  "(PLACE 4)) "
  #  "(AND"
  #  "(PLACE 5) "
  #  "(PLACE 6)))) "
  # )

  # parse_str = parse(string)
  # parse_str2 = parse(string2)

  

  # # outfilestream.write(str(parse_str))
  # # outfilestream.write(str(parse_str2))
  # # outfilestream.close()
  # print("PARSE STR\n\n\n\n")
  # print(parse_str)
  # outfilestream.write(str(parse_str))
  # x = eval(parse_str, pr2_env)
  # x2 = eval(parse_str2, baxter_env)
  # print("XXXXXXXXXXXXXXXXXXX")
  # print(str(x))
  # # outfilestream.write(str(x))
  # # outfilestream.write(str(x2))
  # outfilestream.close()
