#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import quaternion
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
camera = p.getDebugVisualizerCamera()
projMat = camera[3]
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

  cam_pos, cam_quat = p.getLinkState(RobotId, index['camera_link'])[:2]
  quat = np.quaternion(cam_quat[3], cam_quat[0], cam_quat[1], cam_quat[2])
  rotMat = quaternion.as_rotation_matrix(quat)
  pos0 = rotMat[0][0] * cam_pos[0] + rotMat[1][0] * cam_pos[1] + rotMat[2][0] * cam_pos[2];
  pos1 = rotMat[0][1] * cam_pos[0] + rotMat[1][1] * cam_pos[1] + rotMat[2][1] * cam_pos[2];
  pos2 = rotMat[0][2] * cam_pos[0] + rotMat[1][2] * cam_pos[1] + rotMat[2][2] * cam_pos[2];
  viewMat = [rotMat[0][0], rotMat[0][1], rotMat[0][2], 0.0, rotMat[1][0], rotMat[1][1], rotMat[1][2], 0.0, rotMat[2][0], rotMat[2][1], rotMat[2][2], 0.0, -pos0, -pos1, -pos2, 1.0]
  p.getCameraImage(128, 128, renderer=p.ER_BULLET_HARDWARE_OPENGL,
    flags=p.ER_NO_SEGMENTATION_MASK, viewMatrix=viewMat, projectionMatrix=projMat)

  p.stepSimulation()
  sleep(TIME_STEP)

