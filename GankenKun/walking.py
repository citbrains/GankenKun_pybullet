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
    self.foot_step = [[0.0, 0.0, 0.0,'both'], [0.34, 0.0, 0.06,'left'], [0.68, 0.05, -0.06,'right'], [1.02, 0.10, 0.06,'left'], [1.36, 0.15, -0.06,'right']]
    self.left_up = self.right_up = 0.0
    self.is_first = True
    self.gyro_pitch = self.gyro_roll = 0.0
    self.left_off,  self.left_off_g,  self.left_off_d  = np.matrix([[0.0, 0.0]]),  np.matrix([[0.0, 0.0]]),  np.matrix([[0.0, 0.0]]) 
    self.right_off, self.right_off_g, self.right_off_d = np.matrix([[0.0, 0.0]]),  np.matrix([[0.0, 0.0]]),  np.matrix([[0.0, 0.0]])
    return

  def setGoalPos(self, pos):
    if self.is_first == False:
      del self.foot_step[0]
      td = self.foot_step[0][0]
      xd = self.foot_step[0][1]
      for i in range(len(self.foot_step)):
        self.foot_step[i] = [round(self.foot_step[i][0]-td,2), round(self.foot_step[i][1],2), round(self.foot_step[i][2],2), self.foot_step[i][3]]
    self.is_first = False
    self.foot_step += [[round(self.foot_step[4][0]+0.34,2), round(self.foot_step[4][1]+0.05,2), -self.foot_step[4][2], 'right' if self.foot_step[4][3]=='left' else 'left']]
    
    print(str(self.foot_step)+'\n')
    self.X[0] = [self.X[0][0]-self.foot_step[0][1], self.X[0][1]]
    self.pattern, x, y = self.pc.set_param(0,self.X[0], self.X[1], self.X[2], self.foot_step)
    self.X = [[x[0,0], y[0,0]], [x[1,0], y[1,0]], [x[2,0], y[2,0]]]
    if self.foot_step[0][3] == 'left':
      self.right_off_g = np.matrix([[self.foot_step[1][1] - self.foot_step[0][1], self.foot_step[1][2] - self.foot_step[0][2]+0.12]])
      self.left_off, self.left_off_g  = np.matrix([[0.0, 0.0]]), np.matrix([[0.0, 0.0]])
      self.right_off_d = self.right_off_g / 28
    if self.foot_step[0][3] == 'right':
      self.left_off_g  = np.matrix([[self.foot_step[1][1] - self.foot_step[0][1], self.foot_step[1][2] - self.foot_step[0][2]-0.12]])
      self.right_off_g, self.right_off_g  = np.matrix([[0.0, 0.0]]), np.matrix([[0.0, 0.0]])
      self.left_off_d  = self.left_off_g / 28
    print(self.left_off_g, self.right_off_g)
#    self.left_off_g = [foot_step[1][0], foot_step[1][1]]
    return self.pattern

  def getNextPos(self):
    X = self.pattern.pop(0)
    period = round(self.foot_step[1][0]/0.01)
    x_dir = 0
    BOTH_FOOT = round(0.06/0.01)
    start_up = round(BOTH_FOOT/2)
    end_up   = round(period/2)
    period_up = end_up - start_up
    foot_hight = 0.04
    if self.foot_step[0][2] < 0:
      if start_up < (period-len(self.pattern)) <= end_up:
        self.left_up  += foot_hight/period_up
      elif self.left_up > 0:
        self.left_up  = max(self.left_up  - foot_hight/period_up, 0.0)
      if start_up < (period-len(self.pattern)):
        self.left_off += self.left_off_d
        if (period-len(self.pattern)) > (start_up + period_up):
          self.left_off = self.left_off_g.copy()
    if self.foot_step[0][2] > 0:
      if start_up < (period-len(self.pattern)) <= end_up:
        self.right_up += foot_hight/period_up
      elif self.right_up > 0:
        self.right_up = max(self.right_up - foot_hight/period_up, 0.0)
      else:
        self.right_up = 0.0
      if start_up < (period-len(self.pattern)):
        self.right_off += self.right_off_d
        if (period-len(self.pattern)) > (start_up + period_up):
          self.right_off = self.right_off_g.copy()
    lo = self.left_off  - X[0,0:2]
    ro = self.right_off - X[0,0:2]
    left_foot  = [self. left_foot0[0]+lo[0,0], self. left_foot0[1]+lo[0,1], self. left_foot0[2]+self.left_up , 0.0, 0.0, 0.0]
    right_foot = [self.right_foot0[0]+ro[0,0], self.right_foot0[1]+ro[0,1], self.right_foot0[2]+self.right_up, 0.0, 0.0, 0.0]
    self.joint_angles = self.kine.solve_ik(left_foot, right_foot, self.joint_angles)
    xp = [X[0,2], X[0,3]]
    #gyro feedback
    pos,ori,loc_pos,dummy,dummy,loc_ori,vel,ang_vel = p.getLinkState(RobotId, index[ 'body_link'],1)
#    body_angles = p.getEulerFromQuaternion(loc_ori)
#    self.gyro_pitch = 0.9 * self.gyro_pitch + 0.1 * ang_vel[1] * 0.2 # <- TODO:convert
#    self.gyro_roll  = 0.0 * self.gyro_roll  + 1.0 * ang_vel[0] * 0.05
    self.joint_angles[index_dof['left_ankle_roll_link' ]] += self.gyro_roll
    self.joint_angles[index_dof['right_ankle_roll_link']] += self.gyro_roll
    return self.joint_angles, left_foot, right_foot, xp, len(self.pattern)

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

  pc = preview_control(0.01, 1.0, 0.25)

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
      joint_angles,lf,rf,xp,n = walk.getNextPos()
      with open('result.csv', mode='a') as f:
        writer = csv.writer(f)
        writer.writerow(np.concatenate([lf, rf, xp]))
      j = 0
      if n == 0:
        walk.setGoalPos(0)
        with open('result.csv', mode='a') as f:
          f.write('\n')
    for id in range(p.getNumJoints(RobotId)):
      qIndex = p.getJointInfo(RobotId, id)[3]
      if qIndex > -1:
        p.setJointMotorControl2(RobotId, id, p.POSITION_CONTROL, joint_angles[qIndex-7])

    p.stepSimulation()
#    sleep(0.01)
    sleep(TIME_STEP)
