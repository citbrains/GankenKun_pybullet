#!/usr/bin/env python3

import pybullet as p
import numpy as np
import sys
sys.path.append('./GankenKun')
from kinematics import *
from foot_step_planner import *
from preview_control import *
from walking import *
from time import sleep
import csv

if __name__ == '__main__':
  TIME_STEP = 0.001
  physicsClient = p.connect(p.GUI)
  p.setGravity(0, 0, -9.8)
  p.setTimeStep(TIME_STEP)

  planeId = p.loadURDF("URDF/plane.urdf", [0, 0, 0])
  RobotId = p.loadURDF("URDF/gankenkun.urdf", [0, 0, 0])

  index = {p.getBodyInfo(RobotId)[0].decode('UTF-8'):-1,}
  for id in range(p.getNumJoints(RobotId)):
    index[p.getJointInfo(RobotId, id)[12].decode('UTF-8')] = id

  left_foot0  = p.getLinkState(RobotId, index[ 'left_foot_link'])[0]
  right_foot0 = p.getLinkState(RobotId, index['right_foot_link'])[0]

  joint_angles = []
  for id in range(p.getNumJoints(RobotId)):
    if p.getJointInfo(RobotId, id)[3] > -1:
      joint_angles += [0,]

  left_foot  = [ left_foot0[0]-0.015,  left_foot0[1]+0.01,  left_foot0[2]+0.02]
  right_foot = [right_foot0[0]-0.015, right_foot0[1]-0.01, right_foot0[2]+0.02]

  pc = preview_control(0.01, 1.0, 0.27)
  walk = walking(RobotId, left_foot, right_foot, joint_angles, pc)

  index_dof = {p.getBodyInfo(RobotId)[0].decode('UTF-8'):-1,}
  for id in range(p.getNumJoints(RobotId)):
    index_dof[p.getJointInfo(RobotId, id)[12].decode('UTF-8')] = p.getJointInfo(RobotId, id)[3] - 7

  #goal position (x, y) theta
  walk.setGoalPos([0.4, 0.0, 0.5])
  j = 0
  while p.isConnected():
    j += 1
    if j >= 10:
      joint_angles,lf,rf,xp,n = walk.getNextPos()
      j = 0
      if n == 0:
        foot_step = walk.setGoalPos()
        #if you want new goal, please send position
    for id in range(p.getNumJoints(RobotId)):
      qIndex = p.getJointInfo(RobotId, id)[3]
      if qIndex > -1:
        p.setJointMotorControl2(RobotId, id, p.POSITION_CONTROL, joint_angles[qIndex-7])

    p.stepSimulation()
    sleep(TIME_STEP) # delete -> speed up
