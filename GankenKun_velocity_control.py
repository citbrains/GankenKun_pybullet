#!/usr/bin/env python3

import pybullet as p
import numpy as np
import sys
sys.path.append('./GankenKun')
from kinematics import *
from foot_step_planner_v2 import *
from preview_control_v2 import *
from walking_v2 import *
from random import random
from time import sleep
import csv

if __name__ == '__main__':
    # Initialize PyBullet
    TIME_STEP = 0.001
    physicsClient = p.connect(p.GUI)
    p.setGravity(0, 0, -9.8)
    p.setTimeStep(TIME_STEP)

    planeId = p.loadURDF("./URDF/plane.urdf", [0, 0, 0])
    RobotId = p.loadURDF("./URDF/gankenkun.urdf", [0, 0, 0])

    index = {p.getBodyInfo(RobotId)[0].decode('UTF-8'): -1, }
    for id in range(p.getNumJoints(RobotId)):
        index[p.getJointInfo(RobotId, id)[12].decode('UTF-8')] = id

    kine = kinematics(RobotId)

    left_ank_roll0 = p.getLinkState(RobotId, index['left_ankle_roll_link'])[0]
    left_ank_pitch0 = p.getLinkState(
        RobotId, index['left_ankle_pitch_link'])[0]

    joint_angles = []
    for id in range(p.getNumJoints(RobotId)):
        if p.getJointInfo(RobotId, id)[3] > -1:
            joint_angles += [0, ]

    preview_t = 1.5
    pc_dt = 0.0015
    sys_dt = 0.001

    fsp = foot_step_planner(dt=sys_dt, n_steps=4, dsp_ratio=0.15,
                            t_step=0.34, foot_separation=0.044)
    pc = preview_control(dt=pc_dt, preview_t=preview_t)
    walk = walking(pc, fsp,
                   foot_offset=[-0.015, 0.01, 0.02], dt=sys_dt)

    index_dof = {p.getBodyInfo(RobotId)[0].decode('UTF-8'): -1, }

    for id in range(p.getNumJoints(RobotId)):
        index_dof[p.getJointInfo(RobotId, id)[12].decode(
            'UTF-8')] = p.getJointInfo(RobotId, id)[3] - 7

    walk.setVelocity([0, 0, 0.0])
    foot_step = [0, ] * 10

    while p.isConnected():

        # Get current input and output for each step
        _, left_foot_traj, right_foot_traj = walk.get_walk_pattern()

        joint_angles = kine.solve_ik(
            left_foot_traj, right_foot_traj, joint_angles)

        # Change velocity every 10 steps
        if walk.steps_count % 11 == 0:
            if walk.steps_count < 200:
                vel_x = np.random.uniform(0, 0.1)
                vel_y = np.random.uniform(-0.1, 0.1)
                vel_th = np.random.uniform(-0.1, 0.1)
            else:
                vel_x = 0
                vel_y = 0
                vel_th = 0
            walk.setVelocity([vel_x, vel_y, vel_th])

        # Output Target w.r.t hip center
        for id in range(p.getNumJoints(RobotId)):
            qIndex = p.getJointInfo(RobotId, id)[3]
            if qIndex > -1:
                p.setJointMotorControl2(
                    RobotId, id, p.POSITION_CONTROL, joint_angles[qIndex - 7])

        p.stepSimulation()
