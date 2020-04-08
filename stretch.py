#!/usr/bin/env python3

import pybullet as p
from time import sleep

TIME_STEP = 0.001

physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
p.setTimeStep(TIME_STEP)

planeId = p.loadURDF("URDF/plane.urdf", [0, 0, 0])
RobotId = p.loadURDF("URDF/gankenkun.urdf", [0, 0, 0])

for joint in range(p.getNumJoints(RobotId)):
  p.setJointMotorControl(RobotId, joint, p.POSITION_CONTROL, 0)

index = {p.getBodyInfo(RobotId)[0].decode('UTF-8'):-1,}
for id in range(p.getNumJoints(RobotId)):
	index[p.getJointInfo(RobotId, id)[12].decode('UTF-8')] = id

print(index)

p.setJointMotorControl(RobotId, index['left_lower_arm_link' ], p.POSITION_CONTROL, -0.5)
p.setJointMotorControl(RobotId, index['right_lower_arm_link'], p.POSITION_CONTROL, -0.5)
p.setJointMotorControl(RobotId, index['left_upper_arm_link' ], p.POSITION_CONTROL, -0.2)
p.setJointMotorControl(RobotId, index['right_upper_arm_link'], p.POSITION_CONTROL, -0.2)

angle = 0
velocity = 5
while p.isConnected():
  angle += velocity * TIME_STEP
  p.setJointMotorControl(RobotId, index['left_waist_pitch_link' ], p.POSITION_CONTROL, -angle)
  p.setJointMotorControl(RobotId, index['left_shin_pitch_link'  ], p.POSITION_CONTROL,  angle)
  p.setJointMotorControl(RobotId, index['left_knee_pitch_link'  ], p.POSITION_CONTROL,  angle)
  p.setJointMotorControl(RobotId, index['left_ankle_pitch_link' ], p.POSITION_CONTROL, -angle)
  p.setJointMotorControl(RobotId, index['right_waist_pitch_link'], p.POSITION_CONTROL, -angle)
  p.setJointMotorControl(RobotId, index['right_shin_pitch_link' ], p.POSITION_CONTROL,  angle)
  p.setJointMotorControl(RobotId, index['right_knee_pitch_link' ], p.POSITION_CONTROL,  angle)
  p.setJointMotorControl(RobotId, index['right_ankle_pitch_link'], p.POSITION_CONTROL, -angle)
  p.setJointMotorControl(RobotId, index['left_shin_pitch_mimic_link'  ], p.POSITION_CONTROL,  angle)
  p.setJointMotorControl(RobotId, index['left_waist_pitch_mimic_link' ], p.POSITION_CONTROL, -angle)
  p.setJointMotorControl(RobotId, index['right_shin_pitch_mimic_link' ], p.POSITION_CONTROL,  angle)
  p.setJointMotorControl(RobotId, index['right_waist_pitch_mimic_link'], p.POSITION_CONTROL, -angle)
  if angle >= 1.5:
    velocity *= -1.0
  if angle <= 0.0:
    velocity *= -1.0
  p.stepSimulation()
  sleep(TIME_STEP)

