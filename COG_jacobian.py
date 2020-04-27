#!/usr/bin/env python3

import pybullet as p
import numpy as np
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

index_dof = {p.getBodyInfo(RobotId)[0].decode('UTF-8'):-1,}
for id in range(p.getNumJoints(RobotId)):
  index_dof[p.getJointInfo(RobotId, id)[12].decode('UTF-8')] = p.getJointInfo(RobotId, id)[3] - 7

p.setJointMotorControl(RobotId, index['left_lower_arm_link' ], p.POSITION_CONTROL, -0.5)
p.setJointMotorControl(RobotId, index['right_lower_arm_link'], p.POSITION_CONTROL, -0.5)
p.setJointMotorControl(RobotId, index['left_upper_arm_link' ], p.POSITION_CONTROL, -0.2)
p.setJointMotorControl(RobotId, index['right_upper_arm_link'], p.POSITION_CONTROL, -0.2)

left_foot_pos0,  left_foot_ori0  = p.getLinkState(RobotId, index['left_foot_link' ])[:2]
right_foot_pos0, right_foot_ori0 = p.getLinkState(RobotId, index['right_foot_link'])[:2]

initialAngle = []
for id in range(p.getNumJoints(RobotId)):
  if p.getJointInfo(RobotId, id)[3] > -1:
    initialAngle += [0,]

initialAngle[index_dof['left_waist_pitch_link' ]] = -0.5
initialAngle[index_dof['left_shin_pitch_link'  ]] =  0.5
initialAngle[index_dof['left_knee_pitch_link'  ]] =  0.5
initialAngle[index_dof['left_ankle_pitch_link' ]] = -0.5
initialAngle[index_dof['right_waist_pitch_link']] = -0.5
initialAngle[index_dof['right_shin_pitch_link' ]] =  0.5
initialAngle[index_dof['right_knee_pitch_link' ]] =  0.5
initialAngle[index_dof['right_ankle_pitch_link']] = -0.5

height = 0
velocity = 0.2
joint_angle = initialAngle
while p.isConnected():
  height += velocity * TIME_STEP

  # calculate inverse kinematics
  body_pos, body_ori = p.getLinkState(RobotId, index['body_link'])[:2]
  tar_left_foot_pos  = (left_foot_pos0[0] , left_foot_pos0[1] , height)
  tar_right_foot_pos = (right_foot_pos0[0], right_foot_pos0[1], height)
  joint_angle = list(p.calculateInverseKinematics(RobotId, index['left_foot_link' ], tar_left_foot_pos , left_foot_ori0 , currentPositions=joint_angle))
  joint_angle = list(p.calculateInverseKinematics(RobotId, index['right_foot_link'], tar_right_foot_pos, right_foot_ori0, currentPositions=joint_angle))
  joint_angle[index_dof['left_waist_pitch_mimic_link' ]] =  joint_angle[index_dof['left_waist_pitch_link' ]]
  joint_angle[index_dof['left_knee_pitch_link'        ]] = -joint_angle[index_dof['left_waist_pitch_link' ]]
  joint_angle[index_dof['left_shin_pitch_mimic_link'  ]] =  joint_angle[index_dof['left_shin_pitch_link'  ]]
  joint_angle[index_dof['left_independent_pitch_link' ]] = -joint_angle[index_dof['left_shin_pitch_link'  ]]
  joint_angle[index_dof['right_waist_pitch_mimic_link']] =  joint_angle[index_dof['right_waist_pitch_link']]
  joint_angle[index_dof['right_knee_pitch_link'       ]] = -joint_angle[index_dof['right_waist_pitch_link']]
  joint_angle[index_dof['right_shin_pitch_mimic_link' ]] =  joint_angle[index_dof['right_shin_pitch_link' ]]
  joint_angle[index_dof['right_independent_pitch_link']] = -joint_angle[index_dof['right_shin_pitch_link' ]]

  # calculate COG jacobian
  mass = 0.0
  for id in range(p.getNumJoints(RobotId)):
    mass += p.getDynamicsInfo(RobotId, id)[0]
  left_foot_links = [
    [['left_waist_yaw_link',1]],
    [['left_waist_roll_link',2]],
    [['left_waist_pitch_link',3], ['left_knee_pitch_link',3], ['left_waist_pitch_mimic_link',3]],
    [['left_shin_pitch_link',4], ['left_independent_pitch_link',4], ['left_shin_pitch_mimic_link',4]],
    [['left_ankle_pitch_link',5]],
    [['left_ankle_roll_link',6]]]
  zero = [0.0] * len(joint_angle)
  COG_jaco = np.zeros((6,6))
  for links in left_foot_links:
    for link in links:
      pos = p.getLinkState(RobotId, index[link[0]])[2]
      jaco_t, jaco_r = np.array(p.calculateJacobian(
        RobotId, index[link[0]], pos, joint_angle, zero, zero))
      extract_element = [False] * len(jaco_t[0])
      jaco_t[:,index_dof['left_knee_pitch_link']+6] -= jaco_t[:,index_dof['left_waist_pitch_link']+6]
      jaco_r[:,index_dof['left_knee_pitch_link']+6] -= jaco_r[:,index_dof['left_waist_pitch_link']+6]
      jaco_t[:,index_dof['left_independent_pitch_link']+6] -= jaco_t[:,index_dof['left_shin_pitch_link']+6]
      jaco_r[:,index_dof['left_independent_pitch_link']+6] -= jaco_r[:,index_dof['left_shin_pitch_link']+6]
      for link_names in left_foot_links:
        if links[0][0] == link_names[0][0]:
          jaco_t[:,index_dof[link_names[0][0]]+6] = jaco_t[:,index_dof[link[0]]+6]
          jaco_r[:,index_dof[link_names[0][0]]+6] = jaco_r[:,index_dof[link[0]]+6]
        if link[1] >= 4:
          jaco_t[:,index_dof['left_waist_pitch_link']+6] = jaco_t[:,index_dof['left_knee_pitch_link']+6]
          jaco_r[:,index_dof['left_waist_pitch_link']+6] = jaco_r[:,index_dof['left_knee_pitch_link']+6]
        if link[1] >= 5:
          jaco_t[:,index_dof['left_shin_pitch_link']+6] = jaco_t[:,index_dof['left_independent_pitch_link']+6]
          jaco_r[:,index_dof['left_shin_pitch_link']+6] = jaco_r[:,index_dof['left_independent_pitch_link']+6]
        extract_element[index_dof[link_names[0][0]]+6] = True
      jaco = np.concatenate([jaco_t[:,extract_element],jaco_r[:,extract_element]])
      np.set_printoptions(precision=6, floatmode='fixed', suppress=True)
      COG_jaco += jaco*p.getDynamicsInfo(RobotId, index[link[0]])[0]/mass*jaco
  np.set_printoptions(precision=6, floatmode='fixed', suppress=True)
  print(COG_jaco)
  print("\n")
  
  # motor control
  for id in range(p.getNumJoints(RobotId)):
    qIndex = p.getJointInfo(RobotId, id)[3]
    if qIndex > -1:
      p.setJointMotorControl2(RobotId, id, p.POSITION_CONTROL, joint_angle[qIndex-7])

  if height >= 0.1:
    velocity *= -1.0
  if height <= -0.0:
    velocity *= -1.0

  p.stepSimulation()
#  sleep(TIME_STEP)

