#!/usr/bin/env python3
#
# generating walking pattern for the GankenKun

import pybullet as p
import numpy as np
from kinematics import *
from foot_step_planner_v2 import *
from preview_control_v2 import *
from time import sleep
import csv


class walking():
    def __init__(self, pc, fsp=None, foot_offset=[0, 0, 0], dt=0.01):
        self.pc = pc
        self.fsp = fsp if fsp else foot_step_planner(
            0.05, 0.03, 0.2, 0.34, 0.06)
        # x,y,z (pos, vel, acc)
        self.X = np.matrix([[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])
        self.pattern = []
        self.left_up = self.right_up = 0.0
        self.left_off, self.left_off_g, self.left_off_d = np.matrix(
            [[0.0, 0.0, 0.0]]), np.matrix([[0.0, 0.0, 0.0]]), np.matrix([[0.0, 0.0, 0.0]])
        self.right_off, self.right_off_g, self.right_off_d = np.matrix(
            [[0.0, 0.0, 0.0]]), np.matrix([[0.0, 0.0, 0.0]]), np.matrix([[0.0, 0.0, 0.0]])
        self.th = 0
        self.status = 'start'
        self.left_is_swing = True
        self.foot_step = []
        self.foot_offset = np.asarray(
            foot_offset, dtype=np.float32).reshape((3, 1))
        self.leg_separation = 0.044 + foot_offset[1]
        self.cmd_vel = np.zeros((3, 1))
        self.t_sim = 0
        self.dt_sim = dt
        self.z_step_height = 0.06

        # Plan N steps
        self.plan_multi = False
        self.zmp_horizon = []

        # Frame containers
        self.init_supp_position = np.zeros(
            (3, 1), dtype=np.float32)  # x,y,theta
        self.target_supp_position = np.zeros(
            (3, 1), dtype=np.float32)  # x,y,theta

        self.init_swing_position = np.zeros(
            (3, 1), dtype=np.float32)  # x,y,theta
        self.target_swing_position = np.zeros(
            (3, 1), dtype=np.float32)  # x,y,theta

        self.init_torso_position = np.zeros(
            (3, 1), dtype=np.float32)  # x,y,theta
        self.target_torso_position = np.zeros(
            (3, 1), dtype=np.float32)  # x,y,theta

        self.left_foot_traj = np.zeros((4, 1), dtype=np.float32)  # x,y,theta,z
        self.right_foot_traj = np.zeros(
            (4, 1), dtype=np.float32)  # x,y,theta,z
        self.swing_foot_traj = np.zeros(
            (4, 1), dtype=np.float32)  # x,y,theta,z
        self.torso_traj = np.zeros((4, 1), dtype=np.float32)  # x,y,theta,z

        # Frame matrices w.r.t T[k]
        self.cur_torso = np.array([[0], [0], [0]], dtype=np.float32)
        self.cur_rfoot = self.fsp.pose_relative2d(
            np.array([[0], [-self.leg_separation], [0]], dtype=np.float32), self.cur_torso)
        self.cur_lfoot = self.fsp.pose_relative2d(
            np.array([[0], [self.leg_separation], [0]], dtype=np.float32), self.cur_torso)
        self.curr_supp = np.zeros(
            (3, 1), dtype=np.float32)  # x,y,theta

        if self.left_is_swing:
            self.init_supp_position = self.cur_rfoot
            self.init_swing_position = self.cur_lfoot
        else:
            self.init_supp_position = self.cur_lfoot
            self.init_swing_position = self.cur_rfoot

        self.init_torso_position = self.cur_torso

        # First sway step
        self.zmp_buffer = []
        zmp_pos, _ = self.fsp.compute_initial_zmp(self.init_torso_position)
        self.zmp_buffer = np.array(zmp_pos).squeeze()

        self.target_com_pos = []

        self.x_zmp = np.zeros((3, 1), dtype=np.float32)  # x,y,z
        self.x_com = np.zeros((3, 1), dtype=np.float32)  # x,y,z

        print('Initial torso: \n', self.init_torso_position)
        print('Initial support: \n', self.init_supp_position)
        print('Initial swing: \n', self.init_swing_position)
        print('initial buffer: ', len(self.zmp_buffer))

        # Get current com state
        self.state_x = pc.initStateErr(
            pos=float(self.init_torso_position[0]), e=0)
        self.state_y = pc.initStateErr(
            pos=float(self.init_torso_position[1]), e=0)

    def initializeFoot(self, left_foot, right_foot, torso):
        self.cur_torso = np.asarray(torso, dtype=np.float32).reshape((3, 1))
        self.cur_lfoot = np.asarray(
            left_foot, dtype=np.float32).reshape((3, 1))
        self.cur_rfoot = np.asarray(
            right_foot, dtype=np.float32).reshape((3, 1))

    def setVelocity(self, cmd_vel):
        if isinstance(cmd_vel, list) or isinstance(cmd_vel, tuple):
            self.cmd_vel = np.asarray(
                cmd_vel, dtype=np.float32)

    def swapFoot(self):
        """Switch the foot from left to right or or vice versa
        """

        if self.left_is_swing == True:
            self.left_is_swing = False
            self.init_supp_position = self.cur_lfoot
            self.init_swing_position = self.cur_rfoot

        elif self.left_is_swing == False:
            self.left_is_swing = True
            self.init_supp_position = self.cur_rfoot
            self.init_swing_position = self.cur_lfoot

        self.init_torso_position = self.cur_torso

    def calculateCOMTraj(self, t_time, x_com_2d):

        norm_t_time = t_time / self.fsp.t_step

        init_torso_yaw = self.init_torso_position[2]
        target_torso_yaw = self.target_torso_position[2]

        h_func = self.fsp.calcHfunc(t_time, norm_t_time)
        # torso_yaw = h_func*init_torso_yaw+(1-h_func)*target_torso_yaw
        torso_yaw = h_func * target_torso_yaw + \
            (1 - h_func) * init_torso_yaw  # diff from paper
        self.x_com = np.array(
            [x_com_2d[0], x_com_2d[1], float(torso_yaw)]).reshape((3, 1))

    def calcSwingFoot(self, t_time):
        """Calculate the swing foot trajectory

        Reference: Maximo, Marcos - Omnidirectional ZMP-Based Walking for Humanoid
        Section 3.4 - Swing Foot Trajectory

        Calculate the swing foot trajectory based by interpolating S[k] and S[k+1]
        Parameters
        ----------
        t_time : float
            The time since the beginning of the step
        """

        norm_t_time = t_time / self.fsp.t_step

        h_phase = self.fsp.calcHfunc(t_time, norm_t_time)
        swing_horizontal = h_phase * self.target_swing_position + \
            (1 - h_phase) * self.init_swing_position

        v_func = self.fsp.calcVfunc(t_time, norm_t_time)
        swing_vertical = self.z_step_height * \
            v_func + float(self.foot_offset[2])
        # swing_vertical = self.pc.com_height - \
        #     self.z_step_height * v_func  # diff from paper
        self.swing_foot_traj = np.vstack((swing_horizontal, swing_vertical))

    def calcFootPose(self):
        """Calculate the foot poses
        Reference: Maximo, Marcos - Omnidirectional ZMP-Based Walking for Humanoid
        Section 3.5 - Calculating Feet Poses

        Since they were computed in Support foot frame, we need to realign
        support foot and swing foot w.r.t torso. Also add Z and reorder the array
        """

        # w.r.t Torso[k] for ik and visualization

        if self.left_is_swing:
            rel_rfoot = self.fsp.pose_relative2d(
                self.cur_torso, self.cur_rfoot)  # Not sure
            # self.right_foot_traj = self.fsp.pose_global2d(
            #     rel_rfoot, self.cur_torso)
            self.right_foot_traj = rel_rfoot
            self.right_foot_traj = np.vstack(
                (self.right_foot_traj, self.foot_offset[2], 0, 0))  # x,y,theta,z,r,p
            self.right_foot_traj = self.right_foot_traj[[
                0, 1, 3, 4, 5, 2]]  # x,y,z,r,p,y

            rel_lfoot = self.fsp.pose_relative2d(
                self.cur_torso, self.cur_lfoot)
            # self.left_foot_traj = self.fsp.pose_global2d(
            #     rel_lfoot, self.cur_torso)
            self.left_foot_traj = rel_lfoot
            self.left_foot_traj = np.vstack(
                (self.left_foot_traj, self.swing_foot_traj[3], 0, 0))  # x,y,theta,z,r,p
            self.left_foot_traj = self.left_foot_traj[[
                0, 1, 3, 4, 5, 2]]  # x,y,z,r,p,y

        else:
            rel_lfoot = self.fsp.pose_relative2d(
                self.cur_torso, self.cur_lfoot)
            # self.left_foot_traj = self.fsp.pose_global2d(
            #     rel_lfoot, self.cur_torso)
            self.left_foot_traj = rel_lfoot
            self.left_foot_traj = np.vstack(
                (self.left_foot_traj, self.foot_offset[2], 0, 0))  # x,y,theta,z,r,p
            self.left_foot_traj = self.left_foot_traj[[
                0, 1, 3, 4, 5, 2]]  # x,y,z,r,p,y

            rel_rfoot = self.fsp.pose_relative2d(
                self.cur_torso, self.cur_rfoot)
            # self.right_foot_traj = self.fsp.pose_global2d(
            # rel_rfoot, self.cur_torso)
            self.right_foot_traj = rel_rfoot
            self.right_foot_traj = np.vstack(
                (self.right_foot_traj, self.swing_foot_traj[3], 0, 0))  # x,y,theta,z,r,p
            self.right_foot_traj = self.right_foot_traj[[
                0, 1, 3, 4, 5, 2]]  # x,y,z,r,p,y

        self.torso_traj = np.append(self.cur_torso, self.pc.com_height)
        self.torso_traj = self.torso_traj[[0, 1, 3, 2]]

        # print('Torso', self.torso_traj)
        # print('Right foot w.r.t Torso ', self.right_foot_traj.reshape(1, -1))
        # print('Left foot w.r.t Torso ', self.left_foot_traj.reshape(1, -1))
        # print('#### \n')

    def get_walk_pattern(self):

        # Calculate the target torso and foots
        if self.t_sim == 0:

            # Plan up to N steps
            if self.plan_multi:
                next_supp_leg = 'left' if self.left_is_swing else 'right'
                foot_steps, torso_pos, self.zmp_horizon, _ = self.fsp.calculate_v2(
                    self.cmd_vel, self.init_supp_position, self.init_torso_position, next_supp_leg)

                self.target_torso_position = np.asarray(
                    torso_pos[1][1:4], dtype=np.float32).reshape((3, 1))
                self.target_swing_position = np.asarray(
                    foot_steps[1][1:4], dtype=np.float32).reshape((3, 1))

            else:
                self.target_torso_position, self.target_swing_position = self.fsp.calcTorsoFoot(
                    self.cmd_vel, self.init_supp_position, self.init_torso_position, self.left_is_swing)

            print('Curr Support \n', self.init_supp_position.reshape((1, -1)))
            print('Curr Torso \n', self.init_torso_position.reshape((1, -1)))
            print('Target Torso \n', self.target_torso_position.reshape((1, -1)))
            print('Curr Swing \n', self.init_swing_position.reshape((1, -1)))
            print('Target Swing \n', self.target_swing_position.reshape((1, -1)))

        # Compute zmp reference
        if self.plan_multi:
            zmp_2d = np.asarray(self.zmp_horizon[0]).reshape((1, 2))
            self.zmp_horizon = np.asarray(
                self.zmp_horizon, dtype=np.float32).reshape((-1, 2))
        else:
            zmp_2d = self.fsp.compute_zmp_trajectory_v2(
                self.t_sim, self.init_torso_position[:2], self.target_torso_position[:2], self.init_supp_position[:2])
            zmp_2d = np.asarray(zmp_2d).reshape((1, 2))

        # Update buffer
        self.zmp_buffer = np.vstack((self.zmp_buffer, zmp_2d[0]))

        # Compute preview controller
        self.state_x, zmp_x, _ = self.pc.updateStateErr(
            self.state_x, self.zmp_buffer[:, 0])
        self.state_y, zmp_y, _ = self.pc.updateStateErr(
            self.state_y, self.zmp_buffer[:, 1])
        self.calculateCOMTraj(
            self.t_sim, [float(self.state_x[0][0]), float(self.state_y[0][0])])
        self.calcSwingFoot(self.t_sim)

        if self.left_is_swing == "left":
            self.cur_rfoot = self.init_supp_position
            self.cur_lfoot = self.swing_foot_traj[:3]
            self.cur_torso = self.x_com

        else:
            self.cur_rfoot = self.swing_foot_traj[:3]
            self.cur_lfoot = self.init_supp_position
            self.cur_torso = self.x_com

        # print('Torso', self.cur_torso)
        # print('Right foot ', self.cur_rfoot.reshape(1, -1))
        # print('Left foot ', self.cur_lfoot.reshape(1, -1))
        # print('#### \n')

        self.calcFootPose()

        self.t_sim += self.dt_sim
        # Buffer FIFO: Pop com traj at each iteration
        self.zmp_buffer = self.zmp_buffer[1:]

        if self.t_sim > self.fsp.t_step:
            self.t_sim = 0
            self.swapFoot()
            print("#################### SWITCH #################")

        return self.left_foot_traj, self.right_foot_traj


def walking_plot():

    preview_t = 1.5
    pc_dt = 0.05
    sys_dt = 0.01

    planner = foot_step_planner(dt=sys_dt, n_steps=4)
    pc = preview_control(dt=pc_dt, preview_t=preview_t)

    # Plan N steps
    plan_multi = False

    fig, axs = plt.subplots(2, 2)
    fig.tight_layout(pad=1.0)
    axs = axs.ravel()

    t = 0
    left_is_swing = False
    vel_cmd = [0.2, 0, 0]

    # Container
    current_torso = np.zeros((3, 1), dtype=np.float32)
    if left_is_swing:
        current_supp = np.array(
            [0, -0.03525, 0], dtype=np.float32).reshape(3, 1)
        target_swing_pos = np.array(
            [0, 0.03525, 0], dtype=np.float32).reshape(3, 1)
    else:
        current_supp = np.array(
            [0, 0.03525, 0], dtype=np.float32).reshape(3, 1)
        target_swing_pos = np.array(
            [0, -0.03525, 0], dtype=np.float32).reshape(3, 1)

    target_torso_pos = np.zeros((3, 1), dtype=np.float32)

    # First sway step
    zmp_buffer = []
    zmp_ref = []
    zmp_pos, timer_count = planner.compute_initial_zmp(current_torso)
    zmp_ref = np.asarray(zmp_pos, dtype=np.float32).squeeze()
    zmp_buffer = np.array(zmp_pos).squeeze()

    print('Initial torso: \n', current_torso)
    print('Initial support: \n', current_supp)
    print('Initial swing: \n', target_swing_pos)
    print('initial buffer: ', len(zmp_buffer))

    # Set current state as Xk-1
    com_pos = [current_torso[:2]]
    zmp_error = [current_torso[:2]]

    zmp_horizon = []

    # Get current com state
    state_x = pc.initStateErr(
        pos=float(com_pos[-1][0]), e=0)
    state_y = pc.initStateErr(
        pos=float(com_pos[-1][1]), e=0)

    for i in range(int((planner.t_step * 6) // sys_dt)):

        if t == 0:
            # Plan up to N steps
            if plan_multi:

                next_supp_leg = 'left' if left_is_swing else 'right'
                foot_steps, torso_pos, zmp_horizon, _ = planner.calculate_v2(
                    vel_cmd, current_supp, current_torso, next_supp_leg)

                target_torso_pos = torso_pos[1][1:4]
                target_swing_pos = foot_steps[1][1:4]

            else:
                target_torso_pos, target_swing_pos = planner.calcTorsoFoot(
                    vel_cmd, current_supp, current_torso, left_is_swing)

        # Compute zmp reference
        if plan_multi:
            zmp_2d = np.asarray(zmp_horizon[0]).reshape((1, 2))
            zmp_horizon = np.asarray(
                zmp_horizon, dtype=np.float32).reshape((-1, 2))
        else:
            zmp_2d = planner.compute_zmp_trajectory_v2(
                t, current_torso[:2], target_torso_pos[:2], current_supp[:2])
            zmp_2d = np.asarray(zmp_2d).reshape((1, 2))

        # Update buffer
        zmp_buffer = np.vstack((zmp_buffer, zmp_2d[0]))
        zmp_ref = np.vstack((zmp_ref, zmp_2d[0]))

        # Compute preview controller
        state_x, zmp_x, _ = pc.updateStateErr(state_x, zmp_buffer[:, 0])
        state_y, zmp_y, _ = pc.updateStateErr(state_y, zmp_buffer[:, 1])

        pred_com = np.asarray(
            [state_x[0][0], state_y[0][0]], dtype=np.float32).reshape((2, 1))
        pred_zmp = np.asarray(
            [zmp_x, zmp_y], dtype=np.float32).reshape((2, 1))

        # Different index produces different result
        com_pos.append(pred_com)
        zmp_error.append(pred_zmp)

        # Buffer FIFO
        zmp_buffer = zmp_buffer[1:]
        zmp_horizon = zmp_horizon[1:]

        t += sys_dt
        # print('current t: ', t)
        if t > planner.t_step:
            t = 0
            # switch foot
            if left_is_swing == True:
                left_is_swing = False
            else:
                left_is_swing = True

            current_supp = target_swing_pos
            current_torso = target_torso_pos

    zmp_ref = np.asarray(zmp_ref).squeeze()
    com_pos = np.array(com_pos).squeeze()
    zmp_error = np.array(zmp_error).squeeze()

    axs[0].plot(com_pos[:, 0], label='COM X')
    axs[0].plot(zmp_error[:, 0], label='ZMP X Prediction')
    axs[0].plot(zmp_ref[:, 0], label='ZMP X Reference')
    axs[0].legend(loc='upper left', prop={'size': 10})
    axs[0].set_title('ZMP preview controller - inputs & outputs')
    axs[0].set_ylabel('X (meters)')

    axs[1].plot(com_pos[:, 1], label='COM Y')
    axs[1].plot(zmp_error[:, 1], label='ZMP Y Prediction')
    axs[1].plot(zmp_ref[:, 1], label='ZMP Y Reference')
    axs[1].legend(loc='upper left', prop={'size': 10})
    axs[1].set_title('ZMP preview controller - inputs & outputs')
    axs[1].set_ylabel('Y (meters)')
    plt.show()


def walking_v2():

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

    # Get initial pose
    left_foot0 = p.getLinkState(RobotId, index['left_foot_link'])[0]
    right_foot0 = p.getLinkState(RobotId, index['right_foot_link'])[0]
    torso0 = p.getLinkState(RobotId, index['body_link'])[0]

    left_ank_roll0 = p.getLinkState(RobotId, index['left_ankle_roll_link'])[0]
    left_ank_pitch0 = p.getLinkState(
        RobotId, index['left_ankle_pitch_link'])[0]

    print(f"Left Ankle Pitch: \n", left_ank_pitch0)
    print(f"Left Ankle Roll: \n", left_ank_roll0)

    print(f"Left Ankle Pitch-foot: \n",
          np.asarray(left_ank_pitch0) - np.asarray(left_foot0))
    print(f"Left Ankle Roll-foot: \n",
          np.asarray(left_ank_roll0) - np.asarray(left_foot0))

    print(f"Left Foot: \n", left_foot0)
    print(f"Right Foot: \n", right_foot0)
    print(f"Torso: \n", torso0)

    joint_angles = []
    for id in range(p.getNumJoints(RobotId)):
        if p.getJointInfo(RobotId, id)[3] > -1:
            joint_angles += [0, ]

    # Offset initial pose
    left_foot = [left_foot0[0] - 0.015,
                 left_foot0[1] + 0.01, left_foot0[2] + 0.02]
    right_foot = [right_foot0[0] - 0.015,
                  right_foot0[1] - 0.01, right_foot0[2] + 0.02]

    print(f"Left Foot adjusted: \n", left_foot)
    print(f"Right Foot adjusted: \n", right_foot)
    print(f"Torso: \n", torso0)

    print(f"Diff left foot: {np.asarray(left_foot)-np.asarray(left_foot0)}")
    print(f"Diff right foot: {np.asarray(right_foot)-np.asarray(right_foot0)}")

    preview_t = 1.5
    pc_dt = 0.05
    sys_dt = 0.001

    fsp = foot_step_planner(dt=sys_dt, n_steps=4, foot_separation=0.044)
    pc = preview_control(dt=pc_dt, preview_t=preview_t)
    walk = walking(pc, fsp,
                   foot_offset=[-0.015, 0.0, 0.02], dt=0.001)

    index_dof = {p.getBodyInfo(RobotId)[0].decode('UTF-8'): -1, }

    for id in range(p.getNumJoints(RobotId)):
        index_dof[p.getJointInfo(RobotId, id)[12].decode(
            'UTF-8')] = p.getJointInfo(RobotId, id)[3] - 7

    walk.setVelocity([0.1, 0.0, 0.0])
    foot_step = [0, ] * 10

    while p.isConnected():

        # Get current input and output for each step
        left_foot_traj, right_foot_traj = walk.get_walk_pattern()

        joint_angles = kine.solve_ik(
            left_foot_traj, right_foot_traj, joint_angles)

        # j += 1
        # if j >= 10:
        #     joint_angles, lf, rf, xp, n = walk.getNextPos()
        #     with open('result.csv', mode='a') as f:
        #         writer = csv.writer(f)
        #         writer.writerow(np.concatenate([lf, rf, xp]))
        #     j = 0
        #     if n == 0:
        #         if (len(foot_step) <= 6):
        #             foot_step = walk.setGoalPos(
        #                 [foot_step[0][1] + 0.4, foot_step[0][2] + 0.1, foot_step[0][3] + 0.5])
        #             print("send new target *************************")
        #         else:
        #             foot_step = walk.setGoalPos()
        #         with open('result.csv', mode='a') as f:
        #             f.write('\n')

        # Output Target w.r.t hip center
        for id in range(p.getNumJoints(RobotId)):
            qIndex = p.getJointInfo(RobotId, id)[3]
            if qIndex > -1:
                p.setJointMotorControl2(
                    RobotId, id, p.POSITION_CONTROL, joint_angles[qIndex - 7])

        p.stepSimulation()
        # break


def walking_v1():
    TIME_STEP = 0.001
    physicsClient = p.connect(p.GUI)
    p.setGravity(0, 0, -9.8)
    p.setTimeStep(TIME_STEP)

    planeId = p.loadURDF("./URDF/plane.urdf", [0, 0, 0])
    RobotId = p.loadURDF("./URDF/gankenkun.urdf", [0, 0, 0])

    index = {p.getBodyInfo(RobotId)[0].decode('UTF-8'): -1, }
    for id in range(p.getNumJoints(RobotId)):
        index[p.getJointInfo(RobotId, id)[12].decode('UTF-8')] = id

    left_foot0 = p.getLinkState(RobotId, index['left_foot_link'])[0]
    right_foot0 = p.getLinkState(RobotId, index['right_foot_link'])[0]
    torso0 = p.getLinkState(RobotId, index['body_link'])[0]

    print(f"Left Foot: \n", left_foot0)
    print(f"Right Foot: \n", right_foot0)
    print(f"Torso: \n", torso0)

    joint_angles = []
    for id in range(p.getNumJoints(RobotId)):
        if p.getJointInfo(RobotId, id)[3] > -1:
            joint_angles += [0, ]

    # Offset initial pose
    left_foot = [left_foot0[0] - 0.015,
                 left_foot0[1] + 0.01, left_foot0[2] + 0.02]
    right_foot = [right_foot0[0] - 0.015,
                  right_foot0[1] - 0.01, right_foot0[2] + 0.02]

    print(f"Left Foot: \n", left_foot)
    print(f"Right Foot: \n", right_foot)
    print(f"Torso: \n", torso0)

    dt = 0.05
    t_step = 0.25
    n_steps = 4
    preview_t = 1.25
    pc = preview_control(dt=dt, preview_t=preview_t, z=0.23)
    fsp = foot_step_planner(n_steps=4, dt=0.01)
    walk = walking(RobotId, joint_angles, pc, fsp, foot_offset=0.015, dt=0.01)

    index_dof = {p.getBodyInfo(RobotId)[0].decode('UTF-8'): -1, }

    for id in range(p.getNumJoints(RobotId)):
        index_dof[p.getJointInfo(RobotId, id)[12].decode(
            'UTF-8')] = p.getJointInfo(RobotId, id)[3] - 7

    walk.setVelocity([0.1, 0.0, 0.0])
    j = 0
    # with open('result.csv', mode='w') as f:
    #     f.write('')
    foot_step = [0, ] * 10
    while p.isConnected():

        # Get current input and output for each step
        walk.get_walk_pattern()

        # self.joint_angles = walk.solve_ik(
        #     left_foot, right_foot, self.joint_angles)
        # xp = [X[0, 2], X[0, 3]]

        print(f"Target swing foot: \n", walk.swing_foot_traj)
        print(f"Target support foot: \n", walk.init_supp_position)

        # return self.joint_angles, left_foot, right_foot, xp, len(self.pattern)

        # j += 1
        # if j >= 10:
        #     joint_angles, lf, rf, xp, n = walk.getNextPos()
        #     with open('result.csv', mode='a') as f:
        #         writer = csv.writer(f)
        #         writer.writerow(np.concatenate([lf, rf, xp]))
        #     j = 0
        #     if n == 0:
        #         if (len(foot_step) <= 6):
        #             foot_step = walk.setGoalPos(
        #                 [foot_step[0][1] + 0.4, foot_step[0][2] + 0.1, foot_step[0][3] + 0.5])
        #             print("send new target *************************")
        #         else:
        #             foot_step = walk.setGoalPos()
        #         with open('result.csv', mode='a') as f:
        #             f.write('\n')

        # Output Target w.r.t hip center
        # for id in range(p.getNumJoints(RobotId)):
        #     qIndex = p.getJointInfo(RobotId, id)[3]
        #     if qIndex > -1:
        #         p.setJointMotorControl2(
        #             RobotId, id, p.POSITION_CONTROL, joint_angles[qIndex - 7])

        p.stepSimulation()
#    sleep(0.01)
#    sleep(TIME_STEP)


if __name__ == '__main__':
    walking_v2()
