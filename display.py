#!/usr/bin/env python3

import pybullet as p
from time import sleep

TIME_STEP = 0.001

physicsClient = p.connect(p.GUI)
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
p.setGravity(0, 0, -9.8)
p.setTimeStep(TIME_STEP)

planeOrn = [0,0,0,1]
planeId = p.loadURDF("URDF/plane.urdf", [0,0,0], planeOrn)
RobotId = p.loadURDF("URDF/gankenkun.urdf", [0, 0, 0])

numJoints = p.getNumJoints(RobotId)

for joint in range(numJoints):
  p.setJointMotorControl(RobotId, joint, p.POSITION_CONTROL, 0)

while p.isConnected():
  p.stepSimulation()
  sleep(TIME_STEP)

