#!/usr/bin/env python3

from typing import Union, List, Tuple

import pybullet as p
import numpy as np
from kinematics import *
from foot_step_planner_v2 import *
from preview_control_v2 import *
from time import sleep
import csv

from mpl_toolkits.mplot3d import Axes3D
from pytransform3d.rotations import *
from pytransform3d.trajectories import *
from pytransform3d.trajectories import plot_trajectory
from scipy.spatial.transform import Rotation as R


class walking():
    """Walk controller engine

    Parameters
    ----------
    pc : preview_control
        Preview control engine
    fsp : foot_step_planner
        Foot step and torso planner engine
    foot_offset : List, optional
        Foot offset, by default [0, 0, 0]
    dt : float, optional
        Time step, by default 0.01
    """

    def __init__(self, pc: preview_control, fsp: foot_step_planner, foot_offset: List = [0, 0, 0], dt: float = 0.01):
        self.pc = pc
        self.fsp = fsp
        self.left_is_swing = True
        self.foot_offset = np.asarray(
            foot_offset, dtype=np.float32).reshape((3, 1))
        self.leg_separation = self.fsp.y_sep
        self.cmd_vel = np.zeros((3, 1))
        self.t_sim = 0
        self.dt_sim = dt
        self.z_step_height = 0.04

        # Plan N steps
        self.zmp_horizon = []
        self.steps_count = 1
        self.first_step = True

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
        self.supp_foot_traj = np.zeros(
            (4, 1), dtype=np.float32)  # x,y,theta,z
        self.torso_traj = np.zeros((4, 1), dtype=np.float32)  # x,y,theta,z

        # Initialize
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

        # ZMP Buffer FIFO
        self.zmp_buffer = np.zeros((0, 2), dtype=np.float32)

        self.x_zmp = np.zeros((3, 1), dtype=np.float32)  # x,y,z
        self.x_com = np.zeros((3, 1), dtype=np.float32)  # x,y,z

        # Get current com state
        self.state_x = pc.initStateErr(
            pos=float(self.init_torso_position[0]), e=0)
        self.state_y = pc.initStateErr(
            pos=float(self.init_torso_position[1]), e=0)

        print('Initial torso: \n', self.init_torso_position)
        print('Initial support: \n', self.init_supp_position)
        print('Initial swing: \n', self.init_swing_position)
        print('initial buffer: ', len(self.zmp_buffer))
        print('initial State : ', float(
            self.state_x[0][0]), float(self.state_y[0][0]))

    def setVelocity(self, cmd_vel: Union[List, Tuple]):
        """Set velocity command input

        Parameters
        ----------
        cmd_vel : [type]
            [description]
        """
        if isinstance(cmd_vel, list) or isinstance(cmd_vel, tuple):
            self.cmd_vel = np.asarray(
                cmd_vel, dtype=np.float32)

    def swapFoot(self):
        """Switch the foot from left to right or or vice versa
        """
        if not self.first_step:
            if self.left_is_swing:
                self.left_is_swing = False
                self.init_supp_position = self.cur_lfoot
                self.init_swing_position = self.cur_rfoot

            else:
                self.left_is_swing = True
                self.init_supp_position = self.cur_rfoot
                self.init_swing_position = self.cur_lfoot

            self.init_torso_position = self.target_torso_position

        self.first_step = False

    def calculateCOMTraj(self, t_time: float, x_com_2d: np.ndarray):

        norm_t_time = t_time / self.fsp.t_step

        init_torso_yaw = self.init_torso_position[2]
        target_torso_yaw = self.target_torso_position[2]

        h_func = self.fsp.calcHfunc(t_time, norm_t_time)
        torso_yaw = h_func * target_torso_yaw + \
            (1 - h_func) * init_torso_yaw  # diff from paper
        self.x_com = np.array(
            [x_com_2d[0], x_com_2d[1], float(torso_yaw)]).reshape((3, 1))

    def calcSwingFoot(self, t_time: float):
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

        if self.first_step:
            norm_t_time = 0
            h_phase = 0

        swing_horizontal = h_phase * self.target_swing_position + \
            (1 - h_phase) * self.init_swing_position

        v_func = self.fsp.calcVfunc(t_time, norm_t_time)
        swing_vertical = self.z_step_height * \
            v_func + float(self.foot_offset[2])
        self.swing_foot_traj = np.vstack((swing_horizontal, swing_vertical))
        self.supp_foot_traj = np.vstack(
            (self.init_supp_position, float(self.foot_offset[2])))

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
                self.cur_rfoot, self.cur_torso)
            self.right_foot_traj = rel_rfoot + \
                np.array([float(self.foot_offset[0]), float(-self.foot_offset[1]), 0]
                         ).reshape((3, 1))
            self.right_foot_traj = np.vstack(
                (self.right_foot_traj, float(self.foot_offset[2]), 0, 0))  # x,y,theta,z,r,p
            self.right_foot_traj = self.right_foot_traj[[
                0, 1, 3, 4, 5, 2]]  # x,y,z,r,p,y

            rel_lfoot = self.fsp.pose_relative2d(
                self.cur_lfoot, self.cur_torso)
            self.left_foot_traj = rel_lfoot + \
                np.array([float(self.foot_offset[0]), float(self.foot_offset[1]), 0]
                         ).reshape((3, 1))
            self.left_foot_traj = np.vstack(
                (self.left_foot_traj, self.swing_foot_traj[3], 0, 0))  # x,y,theta,z,r,p
            self.left_foot_traj = self.left_foot_traj[[
                0, 1, 3, 4, 5, 2]]  # x,y,z,r,p,y

        else:
            rel_lfoot = self.fsp.pose_relative2d(
                self.cur_lfoot, self.cur_torso)
            self.left_foot_traj = rel_lfoot + \
                np.array([float(self.foot_offset[0]), float(self.foot_offset[1]), 0]
                         ).reshape((3, 1))
            self.left_foot_traj = np.vstack(
                (self.left_foot_traj, float(self.foot_offset[2]), 0, 0))  # x,y,theta,z,r,p
            self.left_foot_traj = self.left_foot_traj[[
                0, 1, 3, 4, 5, 2]]  # x,y,z,r,p,y

            rel_rfoot = self.fsp.pose_relative2d(
                self.cur_rfoot, self.cur_torso)
            self.right_foot_traj = rel_rfoot + \
                np.array([float(self.foot_offset[0]), float(-self.foot_offset[1]), 0]
                         ).reshape((3, 1))
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
        """Compute walk pattern at time (t)

        Returns
        -------
        torso_traj: np.ndarray
            Torso pose at time (t) [x,y,z,th]
        left_foot_traj: np.ndarray
            Left foot pose at time (t) [x,y,z,th]
        right_foot_traj: np.ndarray
            Right foot pose at time (t) [x,y,z,th]
        """

        # Calculate the target torso and foots
        if self.t_sim == 0:

            # Plan up to N steps
            next_supp_leg = 'left' if self.left_is_swing else 'right'
            foot_steps, torso_pos, zmp_horizon_, _ = self.fsp.calculate(
                self.cmd_vel, self.init_supp_position, self.init_torso_position, next_supp_leg, sway=self.first_step)

            self.target_torso_position = np.asarray(
                torso_pos[1][1:4], dtype=np.float32).reshape((3, 1))
            self.target_swing_position = np.asarray(
                foot_steps[1][1:4], dtype=np.float32).reshape((3, 1))

            # Update buffer
            self.zmp_horizon = np.asarray(
                zmp_horizon_, dtype=np.float32).reshape((-1, 2))
            self.zmp_buffer = self.zmp_horizon

            # print('Curr Support \n', self.init_supp_position.reshape((1, -1)))
            # print('Curr Torso \n', self.init_torso_position.reshape((1, -1)))
            # print('Target Torso \n', self.target_torso_position.reshape((1, -1)))
            # print('Curr Swing \n', self.init_swing_position.reshape((1, -1)))
            # print('Target Swing \n', self.target_swing_position.reshape((1, -1)))

        # Compute zmp reference
        zmp_2d = np.asarray(self.zmp_horizon[0]).reshape((1, 2))
        self.zmp_horizon = np.asarray(
            self.zmp_horizon, dtype=np.float32).reshape((-1, 2))

        # Compute preview controller
        self.state_x, zmp_x, _ = self.pc.updateStateErr(
            self.state_x, self.zmp_buffer[:, 0])
        self.state_y, zmp_y, _ = self.pc.updateStateErr(
            self.state_y, self.zmp_buffer[:, 1])
        self.calculateCOMTraj(
            self.t_sim, [float(self.state_x[0][0]), float(self.state_y[0][0])])
        self.calcSwingFoot(self.t_sim)

        if self.left_is_swing:
            self.cur_rfoot = self.init_supp_position
            self.cur_lfoot = self.swing_foot_traj[:3]

        else:
            self.cur_rfoot = self.swing_foot_traj[:3]
            self.cur_lfoot = self.init_supp_position

        self.cur_torso = self.x_com
        self.x_zmp = np.array([float(zmp_x), float(zmp_y), 0]).reshape((3, 1))

        # print('COM: ', float(self.state_x[0][0]), float(self.state_y[0][0]))
        # print('Torso', self.cur_torso)
        # print('Right foot ', self.cur_rfoot.reshape(1, -1))
        # print('Left foot ', self.cur_lfoot.reshape(1, -1))

        self.calcFootPose()

        self.t_sim += self.dt_sim

        # Buffer FIFO: Pop com traj at each iteration
        self.zmp_buffer = self.zmp_buffer[1:]

        if self.t_sim > self.fsp.t_step:
            self.t_sim = 0
            self.steps_count += 1
            self.swapFoot()
            # print("#################### SWITCH #################")

        return self.torso_traj, self.left_foot_traj, self.right_foot_traj


def walking_plot():
    """Plot walking pattern
    """

    preview_t = 1.5
    pc_dt = 0.0015
    sys_dt = 0.001

    planner = foot_step_planner(dt=sys_dt, n_steps=4, dsp_ratio=0.15,
                                t_step=0.34, foot_separation=0.03525)
    pc = preview_control(dt=pc_dt, preview_t=preview_t, z=0.28)

    fig, axs = plt.subplots(2, 2)
    fig.tight_layout(pad=1.0)
    axs = axs.ravel()

    t = 0
    left_is_swing = True
    vel_cmd = [0.2, 0, 0]

    # Container
    current_torso = np.zeros((3, 1), dtype=np.float32)
    target_swing_pos = np.zeros((3, 1), dtype=np.float32)
    if left_is_swing:
        current_supp = np.array(
            [0, -0.03525, 0], dtype=np.float32).reshape(3, 1)
        init_swing_pos = np.array(
            [0, 0.03525, 0], dtype=np.float32).reshape(3, 1)
    else:
        current_supp = np.array(
            [0, 0.03525, 0], dtype=np.float32).reshape(3, 1)
        init_swing_pos = np.array(
            [0, -0.03525, 0], dtype=np.float32).reshape(3, 1)

    target_torso_pos = np.zeros((3, 1), dtype=np.float32)

    # First sway step
    zmp_ref = np.zeros((0, 2), dtype=np.float32)
    zmp_buffer = np.zeros((0, 2), dtype=np.float32)

    print('Initial torso: \n', current_torso)
    print('Initial support: \n', current_supp)
    print('Initial swing: \n', init_swing_pos)
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

    first_step = True

    for i in range(int((planner.t_step * 4) // sys_dt)):

        if t == 0:
            # Plan up to N steps
            next_supp_leg = 'left' if left_is_swing else 'right'
            foot_steps, torso_pos, zmp_horizon, _ = planner.calculate(
                vel_cmd, current_supp, current_torso, next_supp_leg, sway=first_step)

            target_torso_pos = np.asarray(
                torso_pos[1][1:4], dtype=np.float32).reshape((3, 1))
            target_swing_pos = np.asarray(
                foot_steps[1][1:4], dtype=np.float32).reshape((3, 1))

            print('Curr Support \n', current_supp.reshape((1, -1)))
            print('Curr Torso \n', current_torso.reshape((1, -1)))
            print('Target Torso \n', target_torso_pos.reshape((1, -1)))
            print('Curr Swing \n', init_swing_pos.reshape((1, -1)))
            print('Target Swing \n', target_swing_pos.reshape((1, -1)))
            print("########")

            # Update buffer
            zmp_horizon = np.asarray(
                zmp_horizon, dtype=np.float32).reshape((-1, 2))
            zmp_buffer = zmp_horizon.copy()

        # Compute zmp reference
        zmp_2d = np.asarray(zmp_horizon[0]).reshape((1, 2))
        zmp_horizon = np.asarray(
            zmp_horizon, dtype=np.float32).reshape((-1, 2))

        zmp_ref = np.vstack((zmp_ref, zmp_buffer[0]))

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

        # Update time
        t += sys_dt

        if t >= planner.t_step:
            t = 0
            # switch foot
            if not first_step:
                if left_is_swing == True:
                    left_is_swing = False
                else:
                    left_is_swing = True

                init_swing_pos = current_supp.copy()
                current_supp = target_swing_pos
                current_torso = target_torso_pos

            first_step = False

        # Buffer FIFO
        zmp_buffer = zmp_buffer[1:]

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


def walking_sim():
    """PyBullet walk simulator
    """

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

    joint_angles = []
    for id in range(p.getNumJoints(RobotId)):
        if p.getJointInfo(RobotId, id)[3] > -1:
            joint_angles += [0, ]

    # Offset initial pose
    left_foot = [left_foot0[0] - 0.015,
                 left_foot0[1] + 0.01, left_foot0[2] + 0.02]
    right_foot = [right_foot0[0] - 0.015,
                  right_foot0[1] - 0.01, right_foot0[2] + 0.02]

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

    walk.setVelocity([0.1, 0, 0])
    foot_step = [0, ] * 10

    torso_pos = []
    torso_zmp = []
    left_rel_foot_pos = []
    right_rel_foot_pos = []
    swing_foot_pos = []
    supp_foot_pos = []
    left_foot_y = []

    debug = False
    plot_2d = False

    while p.isConnected():

        # Get current input and output for each step
        torso_traj, left_foot_traj, right_foot_traj = walk.get_walk_pattern()
        if debug:
            torso_zmp.append(walk.x_zmp)
            torso_pos.append(torso_traj)
            left_rel_foot_pos.append(left_foot_traj.copy())
            right_rel_foot_pos.append(right_foot_traj.copy())
            swing_foot_pos.append(walk.swing_foot_traj.copy())
            supp_foot_pos.append(walk.supp_foot_traj.copy())

        joint_angles = kine.solve_ik(
            left_foot_traj, right_foot_traj, joint_angles)

        # Change velocity every 10 steps
        if not debug and walk.steps_count % 11 == 0:
            if walk.steps_count < 100:
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

        if debug and walk.steps_count > 10:
            break

    if debug:
        torso_pos = np.asarray(torso_pos).squeeze()
        torso_zmp = np.asarray(torso_zmp).squeeze()
        left_rel_foot_pos = np.array(left_rel_foot_pos).squeeze()
        right_rel_foot_pos = np.array(right_rel_foot_pos).squeeze()
        swing_foot_pos = np.array(swing_foot_pos).squeeze()
        supp_foot_pos = np.array(supp_foot_pos).squeeze()

        # Plot 2D
        if plot_2d:
            fig, axs = plt.subplots(3, 2)
            fig.tight_layout(pad=1.0)
            axs = axs.ravel()

            axs[0].plot(torso_pos[:, 0], label='Torso X')
            axs[0].plot(torso_zmp[:, 0], label='ZMP X')
            axs[0].legend(loc='upper left', prop={'size': 10})
            axs[1].plot(torso_pos[:, 1], label='Torso Y')
            axs[1].plot(torso_zmp[:, 1], label='ZMP Y')
            axs[1].legend(loc='upper left', prop={'size': 10})
            axs[2].plot(left_rel_foot_pos[:, 2], label='Foot Left Z')
            axs[2].plot(right_rel_foot_pos[:, 2], label='Foot Right Z')
            axs[2].legend(loc='upper left', prop={'size': 10})
            axs[3].plot(left_rel_foot_pos[:, 1], label='Foot Left Y')
            axs[3].plot(right_rel_foot_pos[:, 1], label='Foot Right Y')
            axs[3].legend(loc='upper left', prop={'size': 10})
            axs[4].plot(left_rel_foot_pos[:, 0], label='Foot Left X')
            axs[4].plot(right_rel_foot_pos[:, 0], label='Foot Right X')
            axs[4].legend(loc='upper left', prop={'size': 10})

        # Plot 3D
        else:
            fig = plt.figure(1)
            ax = fig.add_subplot(111, projection='3d')
            com_quaternion = np.array([R.from_euler(
                'z', s[3]).as_quat()[[3, 0, 1, 2]] for s in torso_pos], dtype=np.float32)
            com_trajectory = np.hstack((torso_pos[:, :3], com_quaternion))
            zmp_quaternion = np.array([R.from_euler(
                'z', s[3]).as_quat()[[3, 0, 1, 2]] for s in torso_pos], dtype=np.float32)
            zmp_trajectory = np.hstack((torso_zmp[:, : 3], zmp_quaternion))
            swing_quaternion = np.array([R.from_euler(
                'z', s[2]).as_quat()[[3, 0, 1, 2]] for s in swing_foot_pos], dtype=np.float32)
            swing_trajectory = np.hstack(
                (swing_foot_pos[:, :2], swing_foot_pos[:, 3:4], swing_quaternion))
            swing_trajectory = np.array(swing_trajectory, dtype=np.float32)
            supp_quaternion = np.array([R.from_euler(
                'z', s[2]).as_quat()[[3, 0, 1, 2]] for s in supp_foot_pos], dtype=np.float32)
            supp_trajectory = np.hstack(
                (supp_foot_pos[:, :2], supp_foot_pos[:, 3:4], supp_quaternion))
            supp_trajectory = np.array(supp_trajectory, dtype=np.float32)

            # Plot trajectory (x,y,z,qw,qx,qy,qz)
            plot_trajectory(ax=ax, P=com_trajectory,
                            n_frames=com_trajectory.shape[0], s=0.02, show_direction=False)
            plot_trajectory(ax=ax, P=zmp_trajectory,
                            n_frames=zmp_trajectory.shape[0], s=0.02, show_direction=False)
            plot_trajectory(ax=ax, P=swing_trajectory,
                            n_frames=swing_trajectory.shape[0], s=0.02, show_direction=False)
            plot_trajectory(ax=ax, P=supp_trajectory,
                            n_frames=supp_trajectory.shape[0], s=0.02, show_direction=False)
            ax.set_xlim3d(-0.1, 0.5)
            ax.set_ylim3d(-0.1, 0.5)
            ax.set_zlim3d(0, 0.3)

        plt.show()


if __name__ == '__main__':
    walking_sim()
    # walking_plot()
