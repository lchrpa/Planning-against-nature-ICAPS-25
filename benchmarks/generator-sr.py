import sys
import random


def generate(robots,rooms,items):
  print("(define (problem ServiceRobot-problem-b{}r{}i{})".format(robots,rooms, items))
  print("(:domain ServiceRobot)")
  print("(:objects ")
  for y in range(1, rooms+1):
      print("r{}".format(y), end=' ')
  print("- room")
  for y in range(1, robots+1):
      print("b{}".format(y), end=' ')
  print("- robot")
  for y in range(1, items+1):
      print("i{}".format(y), end=' ')
  print("- item")
  for y in range(1, robots+1):
      print("h{}-l h{}-r".format(y,y), end=' ')
  print("- hand")
  print(")")

  itempos=[]
  
  #print("(:init", "(at-robot r1)", "(free-corridor)", sep='\n')
  print("(:init ")
  for y in range(1, robots+1):
      print("(at-robot b{} r{})".format(y,random.randint(1,rooms)), end='\n')
      print("(has-hand b{} h{}-l)".format(y,y),end="\n")
      print("(has-hand b{} h{}-r)".format(y,y),end="\n")
      print("(free h{}-l)".format(y),end="\n")
      print("(free h{}-r)".format(y),end="\n")
  for y in range(1, items+1):
      itempos.append(random.randint(1,rooms))
      print("(at-item i{} r{})".format(y,itempos[y-1]), end='\n')
      if random.randint(0,1) == 0:
      	print("(fragile i{})".format(y), end='\n')
      print("(notbroken i{})".format(y), end='\n')
  print(")")
  
  print("(:goal (and ")
  for y in range(1, items+1):
      rms=list(range(1,rooms+1))
      rms.remove(itempos[y-1])
      print("(at-item i{} r{})".format(y,random.choice(rms)), end='\n')
      print("(notbroken i{})".format(y), end='\n')
  print("))")
  print(")")

try:
  assert len(sys.argv) == 4
  factor = int(sys.argv[1])
  
  generate(int(sys.argv[1]),int(sys.argv[2]),int(sys.argv[3]))
except AssertionError:
  print("usage: python3 HomeRobot-generator.py <robots> <rooms> <items>")
