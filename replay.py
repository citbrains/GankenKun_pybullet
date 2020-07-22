#!/usr/bin/env python3

# fix the base_link to world (modify the URDF file)

import pybullet as p
from time import sleep
import csv
import math

TIME_STEP = 0.001

physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
p.setTimeStep(TIME_STEP)

RobotId = p.loadURDF("URDF/gankenkun.urdf", [0, 0, 0])

for joint in range(p.getNumJoints(RobotId)):
  p.setJointMotorControl(RobotId, joint, p.POSITION_CONTROL, 0)

index = {p.getBodyInfo(RobotId)[0].decode('UTF-8'):-1,}
for id in range(p.getNumJoints(RobotId)):
	index[p.getJointInfo(RobotId, id)[12].decode('UTF-8')] = id

print(index)

with open("./data4.csv") as f:
  reader = csv.reader(f)

  for angle in reader:
    for i in range(10):
      p.setJointMotorControl2(RobotId, index['right_ankle_roll_link'        ], p.POSITION_CONTROL,  math.radians(float(angle[2])))
      p.setJointMotorControl2(RobotId, index['right_ankle_pitch_link'       ], p.POSITION_CONTROL, -math.radians(float(angle[3])))

      p.setJointMotorControl2(RobotId, index['right_shin_pitch_link'        ], p.POSITION_CONTROL, -math.radians(float(angle[4])))
      p.setJointMotorControl2(RobotId, index['right_independent_pitch_link' ], p.POSITION_CONTROL,  math.radians(float(angle[4])))
      p.setJointMotorControl2(RobotId, index['right_shin_pitch_mimic_link'  ], p.POSITION_CONTROL, -math.radians(float(angle[4])))

      p.setJointMotorControl2(RobotId, index['right_knee_pitch_link'        ], p.POSITION_CONTROL,  math.radians(float(angle[5])))
      p.setJointMotorControl2(RobotId, index['right_waist_pitch_link'       ], p.POSITION_CONTROL, -math.radians(float(angle[5])))
      p.setJointMotorControl2(RobotId, index['right_waist_pitch_mimic_link' ], p.POSITION_CONTROL, -math.radians(float(angle[5])))

      p.setJointMotorControl2(RobotId, index['right_waist_roll_link'        ], p.POSITION_CONTROL, -math.radians(float(angle[6])))
      p.setJointMotorControl2(RobotId, index['right_waist_yaw_link'         ], p.POSITION_CONTROL,  math.radians(float(angle[7])))

      p.setJointMotorControl2(RobotId, index['right_shoulder_link'          ], p.POSITION_CONTROL, -math.radians(float(angle[8])))
      p.setJointMotorControl2(RobotId, index['right_upper_arm_link'         ], p.POSITION_CONTROL,  math.radians(float(angle[9])))
      p.setJointMotorControl2(RobotId, index['right_lower_arm_link'         ], p.POSITION_CONTROL,  math.radians(float(angle[10])))

      p.setJointMotorControl2(RobotId, index['left_ankle_roll_link'         ], p.POSITION_CONTROL,  math.radians(float(angle[11])))
      p.setJointMotorControl2(RobotId, index['left_ankle_pitch_link'        ], p.POSITION_CONTROL,  math.radians(float(angle[12])))

      p.setJointMotorControl2(RobotId, index['left_shin_pitch_link'         ], p.POSITION_CONTROL,  math.radians(float(angle[13])))
      p.setJointMotorControl2(RobotId, index['left_independent_pitch_link'  ], p.POSITION_CONTROL, -math.radians(float(angle[13])))
      p.setJointMotorControl2(RobotId, index['left_shin_pitch_mimic_link'   ], p.POSITION_CONTROL,  math.radians(float(angle[13])))

      p.setJointMotorControl2(RobotId, index['left_knee_pitch_link'         ], p.POSITION_CONTROL, -math.radians(float(angle[14])))
      p.setJointMotorControl2(RobotId, index['left_waist_pitch_link'        ], p.POSITION_CONTROL,  math.radians(float(angle[14])))
      p.setJointMotorControl2(RobotId, index['left_waist_pitch_mimic_link'  ], p.POSITION_CONTROL,  math.radians(float(angle[14])))

      p.setJointMotorControl2(RobotId, index['left_waist_roll_link'         ], p.POSITION_CONTROL, -math.radians(float(angle[15])))
      p.setJointMotorControl2(RobotId, index['left_waist_yaw_link'          ], p.POSITION_CONTROL,  math.radians(float(angle[16])))

      p.setJointMotorControl2(RobotId, index['left_shoulder_link'           ], p.POSITION_CONTROL,  math.radians(float(angle[17])))
      p.setJointMotorControl2(RobotId, index['left_upper_arm_link'          ], p.POSITION_CONTROL,  math.radians(float(angle[18])))
      p.setJointMotorControl2(RobotId, index['left_lower_arm_link'          ], p.POSITION_CONTROL, -math.radians(float(angle[19])))

      p.setJointMotorControl2(RobotId, index['head_yaw_link'                ], p.POSITION_CONTROL, -math.radians(float(angle[20])))

      p.stepSimulation()
#      sleep(TIME_STEP)

