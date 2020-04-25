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
velocity = 1.5
while p.isConnected():
  angle += velocity * TIME_STEP
  angleR = 0.5 - angle
  p.setJointMotorControl2(RobotId, index['left_waist_pitch_link' ], p.POSITION_CONTROL, -angle)
  p.setJointMotorControl2(RobotId, index['left_shin_pitch_link'  ], p.POSITION_CONTROL,  angle)
  p.setJointMotorControl2(RobotId, index['left_knee_pitch_link'  ], p.POSITION_CONTROL,  angle)
  p.setJointMotorControl2(RobotId, index['left_ankle_pitch_link' ], p.POSITION_CONTROL, -angle)
  p.setJointMotorControl2(RobotId, index['right_waist_pitch_link'], p.POSITION_CONTROL, -angleR)
  p.setJointMotorControl2(RobotId, index['right_shin_pitch_link' ], p.POSITION_CONTROL,  angleR)
  p.setJointMotorControl2(RobotId, index['right_knee_pitch_link' ], p.POSITION_CONTROL,  angleR)
  p.setJointMotorControl2(RobotId, index['right_ankle_pitch_link'], p.POSITION_CONTROL, -angleR)
  p.setJointMotorControl2(RobotId, index['left_shin_pitch_mimic_link'  ], p.POSITION_CONTROL,  angle)
  p.setJointMotorControl2(RobotId, index['left_waist_pitch_mimic_link' ], p.POSITION_CONTROL, -angle)
  p.setJointMotorControl2(RobotId, index['right_shin_pitch_mimic_link' ], p.POSITION_CONTROL,  angleR)
  p.setJointMotorControl2(RobotId, index['right_waist_pitch_mimic_link'], p.POSITION_CONTROL, -angleR)
  if angle >= 0.5:
    velocity *= -1.0
  if angle <= 0.0:
    velocity *= -1.0

  # display COG
  mass, cog_x, cog_y, cog_z = 0.0, 0.0, 0.0, 0.0
  for id in range(p.getNumJoints(RobotId)):
    mass += p.getDynamicsInfo(RobotId, id)[0]
  for id in range(p.getNumJoints(RobotId)):
    pos = p.getLinkState(RobotId, id)[0]
    mass_p = p.getDynamicsInfo(RobotId, id)[0]
    cog_x += mass_p/mass*pos[0]
    cog_y += mass_p/mass*pos[1]
    cog_z += mass_p/mass*pos[2]
  line_x0, line_x1 = [cog_x-0.1, cog_y, cog_z], [cog_x+0.1, cog_y, cog_z]
  line_y0, line_y1 = [cog_x, cog_y-0.1, cog_z], [cog_x, cog_y+0.1, cog_z]
  line_z0, line_z1 = [cog_x, cog_y, cog_z-0.1], [cog_x, cog_y, cog_z+0.1]
  p.removeAllUserDebugItems()
  p.addUserDebugLine(line_x0, line_x1, [1,0,0], 10)
  p.addUserDebugLine(line_y0, line_y1, [1,0,0], 10)
  p.addUserDebugLine(line_z0, line_z1, [1,0,0], 10)

  p.stepSimulation()
  sleep(TIME_STEP)

