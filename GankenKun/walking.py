#!/usr/bin/env python3
# 
# generating walking pattern for the GankenKun

import pybullet as p
import numpy as np
from kinematics import *
from foot_step_planner import *
from preview_control import *
from time import sleep
import csv

class walking():
  def __init__(self, RobotId, left_foot0, right_foot0, joint_angles, pc):
    self.kine = kinematics(RobotId)
    self.left_foot0, self.right_foot0 = left_foot0, right_foot0
    self.joint_angles = joint_angles
    self.pc = pc
    self.X = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
    self.pattern = []
    self.foot_step = [[0.0, 0.0, 0.0], [0.34, 0.0, 0.06], [0.68, 0.05, -0.06], [1.02, 0.10, 0.06], [1.36, 0.15, -0.06]]
    self.left_up = self.right_up = 0.0
    self.is_first = True
    return

  def setGoalPos(self, pos):
    if self.is_first == False:
      del self.foot_step[0]
      td = self.foot_step[0][0]
      xd = self.foot_step[0][1]
      for i in range(len(self.foot_step)):
        self.foot_step[i] = [self.foot_step[i][0]-td, self.foot_step[i][1]-xd, self.foot_step[i][2]]
    self.is_first = False
    self.foot_step += [[self.foot_step[4][0]+0.34, self.foot_step[4][1]+0.05, -self.foot_step[4][2]]]
#    print(self.foot_step)
    self.X[0] = [self.X[0][0]-self.foot_step[0][1], self.X[0][1]]
#    print(self.X[0])
    self.pattern, x, y = self.pc.set_param(self.X[0], self.X[1], self.X[2], self.foot_step)
    self.X = [[x[0,0], y[0,0]], [x[1,0], y[1,0]], [x[2,0], y[2,0]]]
    return self.pattern

  def getNextPos(self):
    X = self.pattern.pop(0)
    period = int(self.foot_step[1][0]/0.01+0.005)
    x_dir = 0
    if self.foot_step[0][2] > 0:
      if (period-len(self.pattern)) < period/2:
        self.left_up  += 0.04/(period/2)
      elif self.left_up > 0:
        self.left_up  -= 0.04/(period/2)
      else:
        self.left_up  = 0.0
      x_dir = 1
    if self.foot_step[0][2] < 0:
      if (period-len(self.pattern)) < period/2:
        self.right_up += 0.04/(period/2)
      elif self.right_up > 0:
        self.right_up -= 0.04/(period/2)
      else:
        self.right_up = 0.0
      x_dir = -1
#    print(X)
    left_foot  = [self. left_foot0[0]+x_dir*X[0,0], self. left_foot0[1]+X[0,1], self. left_foot0[2]+self.left_up , 0.0, 0.0, 0.0]
    right_foot = [self.right_foot0[0]+x_dir*X[0,0], self.right_foot0[1]+X[0,1], self.right_foot0[2]+self.right_up, 0.0, 0.0, 0.0]
    self.joint_angles = self.kine.solve_ik(left_foot, right_foot, self.joint_angles)
    return self.joint_angles, left_foot, right_foot, len(self.pattern)

if __name__ == '__main__':
  TIME_STEP = 0.001
  physicsClient = p.connect(p.GUI)
  p.setGravity(0, 0, -9.8)
  p.setTimeStep(TIME_STEP)

  planeId = p.loadURDF("../URDF/plane.urdf", [0, 0, 0])
  RobotId = p.loadURDF("../URDF/gankenkun.urdf", [0, 0, 0])

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

  pc = preview_control(0.01, 1.0, 0.29)

  walk = walking(RobotId, left_foot, right_foot, joint_angles, pc)

  index_dof = {p.getBodyInfo(RobotId)[0].decode('UTF-8'):-1,}
  for id in range(p.getNumJoints(RobotId)):
    index_dof[p.getJointInfo(RobotId, id)[12].decode('UTF-8')] = p.getJointInfo(RobotId, id)[3] - 7

  walk.setGoalPos(0)
  j = 0
  with open('result.csv', mode='w') as f:
    f.write('')
  while p.isConnected():
    j += 1
    if j >= 10:
      joint_angles,lf,rf,n = walk.getNextPos()
      with open('result.csv', mode='a') as f:
#        f.writerow(lf)+str(r for r in np.ravel(rf).tolist())+'\n')
        writer = csv.writer(f)
        writer.writerow(np.concatenate([lf, rf]))
      j = 0
      if n == 0:
        walk.setGoalPos(0)
    for id in range(p.getNumJoints(RobotId)):
      qIndex = p.getJointInfo(RobotId, id)[3]
      if qIndex > -1:
        p.setJointMotorControl2(RobotId, id, p.POSITION_CONTROL, joint_angles[qIndex-7])

    p.stepSimulation()
#    sleep(0.01)
#    sleep(TIME_STEP)
