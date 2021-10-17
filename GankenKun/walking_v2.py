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
    def __init__(self, RobotId, joint_angles, pc, fsp, foot_offset, dt=0.01):
        self.kine = kinematics(RobotId)
        self.joint_angles = joint_angles
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
        self.next_leg = 'left'
        self.foot_step = []
        self.foot_offset = foot_offset
        self.cmd_vel = np.zeros((3, 1))
        self.t_sim = 0
        self.dt_sim = dt
        self.z_step_height = 0.06

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

        # Frame matrices w.r.t S[k] or support foot
        self.cur_torso = np.array([[0], [0], [0]], dtype=np.float32)
        self.cur_rfoot = self.fsp.pose_relative2d(
            np.array([[0], [-self.foot_offset], [0]], dtype=np.float32), self.cur_torso)
        self.cur_lfoot = self.fsp.pose_relative2d(
            np.array([[0], [self.foot_offset], [0]], dtype=np.float32), self.cur_torso)
        self.curr_supp = np.zeros(
            (3, 1), dtype=np.float32)  # x,y,theta
        if self.next_leg == "left":
            self.init_supp_position = self.cur_rfoot
        else:
            self.init_supp_position = self.cur_lfoot

        self.target_com_pos = []

        self.x_zmp = np.zeros((3, 1), dtype=np.float32)  # x,y,z
        self.x_com = np.zeros((3, 1), dtype=np.float32)  # x,y,z

    def setVelocity(self, cmd_vel):
        if isinstance(cmd_vel, list) or isinstance(cmd_vel, tuple):
            self.cmd_vel = np.asarray(
                cmd_vel, dtype=np.float32)

    def swapFoot(self):
        """Switch the foot from left to right or or vice versa
        """

        if self.next_leg == 'left':
            self.next_leg = 'right'
            self.init_supp_position = self.cur_lfoot
        elif self.next_leg == 'right':
            self.next_leg = 'left'
            self.init_supp_position = self.cur_rfoot

    def calculateCOMTraj(self, t_time, x_com_2d):

        norm_t_time = t_time / self.fsp.t_step

        init_torso_yaw = self.init_torso_position[2]
        target_torso_yaw = self.target_torso_position[2]

        h_func = self.fsp.calcHfunc(t_time, norm_t_time)
        # torso_yaw = h_func*init_torso_yaw+(1-h_func)*target_torso_yaw
        torso_yaw = h_func * target_torso_yaw + \
            (1 - h_func) * init_torso_yaw  # diff from paper
        self.x_com = np.array(
            [x_com_2d[0, 0], x_com_2d[0, 1], float(torso_yaw)])

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
        # swing_vertical = self.z_step*v_func
        swing_vertical = self.pc.com_height - \
            self.z_step_height * v_func  # diff from paper
        self.swing_foot_traj = np.vstack((swing_horizontal, swing_vertical))

    def calcFootPose(self):
        """Calculate the foot poses
        Reference: Maximo, Marcos - Omnidirectional ZMP-Based Walking for Humanoid
        Section 3.5 - Calculating Feet Poses

        Since they were computed in Support foot frame, we need to realign
        support foot and swing foot w.r.t torso. Also add Z and reorder the array
        """

        # w.r.t Torso[k] for ik and visualization

        if self.next_leg == "left":
            rel_rfoot = self.fsp.pose_relative2d(
                self.cur_rfoot, self.cur_torso)  # Not sure
            self.right_foot_traj = self.fsp.pose_global2d(
                rel_rfoot, self.cur_torso)
            self.right_foot_traj = np.vstack(
                (self.right_foot_traj, 0))  # x,y,theta,z
            self.right_foot_traj = self.right_foot_traj[[0, 1, 3, 2]]

            rel_lfoot = self.fsp.pose_relative2d(
                self.swing_foot_traj[:3], self.cur_torso)
            self.left_foot_traj = self.fsp.pose_global2d(
                rel_lfoot, self.cur_torso)
            self.left_foot_traj = np.vstack(
                (self.left_foot_traj, self.swing_foot_traj[3]))  # x,y,theta,z
            self.left_foot_traj = self.left_foot_traj[[0, 1, 3, 2]]

        else:
            rel_lfoot = self.fsp.pose_relative2d(
                self.cur_lfoot, self.cur_torso)
            self.left_foot_traj = self.fsp.pose_global2d(
                rel_lfoot, self.cur_torso)
            self.left_foot_traj = np.vstack(
                (self.left_foot_traj, 0))  # x,y,theta,z
            self.left_foot_traj = self.left_foot_traj[[0, 1, 3, 2]]

            rel_rfoot = self.fsp.pose_relative2d(
                self.swing_foot_traj[:3], self.cur_torso)
            self.right_foot_traj = self.fsp.pose_global2d(
                rel_rfoot, self.cur_torso)
            self.right_foot_traj = np.vstack(
                (self.right_foot_traj, self.swing_foot_traj[3]))  # x,y,theta,z
            self.right_foot_traj = self.right_foot_traj[[0, 1, 3, 2]]

        self.torso_traj = np.append(self.cur_torso, self.pc.com_height)
        self.torso_traj = self.torso_traj[[0, 1, 3, 2]]

        # print('Torso', self.torso_traj)
        # print('Right foot w.r.t Torso ', self.right_foot_traj)
        # print('Left foot w.r.t Torso ', self.left_foot_traj)
        # print('#### \n')

    def get_walk_pattern(self):

        # Calculate the target torso and foots
        if self.t_sim == 0:
            target_foot_step, target_torso_pos, target_zmp_pos, timer_count = self.fsp.calculate(
                self.cmd_vel, self.init_supp_position, self.cur_torso, self.next_leg)

            target_foot_step = np.asarray(target_foot_step)
            target_zmp_pos = np.asarray(target_zmp_pos, dtype=np.float32)
            target_torso_pos = np.asarray(target_torso_pos[1:4])
            print("torsosxx", target_torso_pos[1, 1:])

            self.target_torso_position = np.array(
                target_torso_pos[1, 1:], dtype=np.float32).reshape((3, 1))
            self.init_swing_position = np.array(
                target_foot_step[0, 1:4], dtype=np.float32).reshape((3, 1))
            self.target_swing_position = np.array(
                target_foot_step[1, 1:4], dtype=np.float32).reshape((3, 1))

            # Set current state as Xk-1
            state_x = pc.initStateErr(
                pos=float(self.init_torso_position[0]), e=0)
            state_y = pc.initStateErr(
                pos=float(self.init_torso_position[1]), e=0)

            cog_list, _, _ = self.pc.update_preview_controller(
                state_x, state_y, target_zmp_pos)
            cog_list = np.asarray(cog_list, dtype=np.float32).squeeze()
            # Only use the first step, except for first step
            if self.status == 'start':
                self.status = 'continue'

                end_com_pos = int(
                    self.fsp.t_end // self.dt_sim)
                print(f'len {len(cog_list)} - requested: {end_com_pos}')
                self.target_com_pos = cog_list[:end_com_pos, :2]
            else:
                start_com_pos = int(
                    (self.fsp.t_end) // self.dt_sim)
                end_com_pos = start_com_pos + int(
                    (self.fsp.t_step) // self.dt_sim)
                self.target_com_pos = cog_list[start_com_pos:end_com_pos]

        self.calculateCOMTraj(self.t_sim, self.target_com_pos)
        self.calcSwingFoot(self.t_sim)

        if self.next_leg == "left":
            self.cur_rfoot = self.init_supp_position
            self.cur_lfoot = self.swing_foot_traj[:3]
            self.cur_torso = self.x_com

        else:
            self.cur_rfoot = self.swing_foot_traj[:3]
            self.cur_lfoot = self.init_supp_position
            self.cur_torso = self.x_com

        self.calcFootPose()

        self.t_sim += self.dt_sim
        # Pop com traj at each iteration
        if len(self.target_com_pos) > 0:
            self.target_com_pos = self.target_com_pos[1:]

        if self.t_sim > self.fsp.t_step or len(self.target_com_pos) == 0:
            self.t_sim = 0
            self.swapFoot()
            print("#################### SWITCH #################")


def walking_v2():

    preview_t = 1.2
    pc_dt = 0.05
    sys_dt = 0.01

    planner = foot_step_planner(dt=sys_dt, n_steps=4)
    pc = preview_control(dt=pc_dt, preview_t=preview_t)
    # walk = walking(RobotId, joint_angles, pc, fsp,
    #                foot_offset=0.03523, dt=0.01)

    fig, axs = plt.subplots(2, 2)
    fig.tight_layout(pad=1.0)
    axs = axs.ravel()

    supp_foot = np.asarray([0, -0.03525, 0]).reshape((3, 1))
    torso = np.asarray([0, 0.0, 0]).reshape((3, 1))

    # First sway step
    zmp_buffer = []
    zmp_ref = []
    zmp_pos, timer_count = planner.compute_initial_zmp(torso)
    zmp_ref = np.asarray(zmp_pos, dtype=np.float32).squeeze()
    zmp_buffer = np.array(zmp_pos).squeeze()

    t = 0
    left_is_swing = True
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

    # Add 1 future steps
    # target_torso_pos, target_swing_pos = planner.calcTorsoFoot(
    #     vel_cmd, current_supp, current_torso, left_is_swing)

    # # Compute zmp
    # zmp_pos, _ = planner.compute_zmp_trajectory(
    #     0, current_torso[:2], target_torso_pos[:2], current_supp[:2])
    # zmp_pos = np.asarray(zmp_pos)
    # zmp_buffer = np.vstack((zmp_buffer, zmp_pos.squeeze()))
    # zmp_ref = np.vstack((zmp_ref, zmp_pos.squeeze()))

    # # switch foot
    # if left_is_swing == True:
    #     left_is_swing = False
    # else:
    #     left_is_swing = True

    # current_supp = target_swing_pos
    # current_torso = target_torso_pos

    # Set current state as Xk-1
    com_pos = [current_torso[:2]]
    zmp_error = [current_torso[:2]]

    print('Initial torso: \n', current_torso)
    print('Initial support: \n', current_supp)
    print('Initial swing: \n', target_swing_pos)
    print('initial buffer: ', len(zmp_buffer))

    for i in range(int((planner.t_step * 2) // sys_dt)):

        if t == 0:
            # Plan the next target steps
            target_torso_pos, target_swing_pos = planner.calcTorsoFoot(
                vel_cmd, current_supp, current_torso, left_is_swing)

            # # Plan up to N steps
            # zmp_horizon = []
            # temp_torso = target_torso_pos
            # temp_supp = target_swing_pos
            # temp_swing = not left_is_swing
            # for i in range(3):
            #     temp_target_torso_pos, temp_target_swing_pos = planner.calcTorsoFoot(
            #         vel_cmd, temp_supp, temp_torso, temp_swing)

            #     temp_zmp, temp_tzmp = planner.compute_zmp_trajectory(
            #         0, temp_torso[:2], temp_target_torso_pos[:2], temp_supp[:2])
            #     zmp_horizon += temp_zmp
            #     temp_supp = temp_target_swing_pos
            #     temp_torso = temp_target_torso_pos
            #     temp_swing = not temp_swing

        # Get current com state
        state_x = pc.initStateErr(pos=float(com_pos[-1][0]), e=0)
        state_y = pc.initStateErr(pos=float(com_pos[-1][1]), e=0)

        # Compute zmp reference
        zmp_2d = planner.compute_zmp_trajectory_v2(
            t, current_torso[:2], target_torso_pos[:2], current_supp[:2])
        zmp_2d = np.asarray(zmp_2d).reshape((1, 2))

        # Update buffer
        zmp_buffer = np.vstack((zmp_buffer, zmp_2d[0]))
        zmp_ref = np.vstack((zmp_ref, zmp_2d[0]))

        # Compute preview controller
        cog_list, _, _ = pc.update_preview_controller(
            state_x, state_y, zmp_buffer)

        cog_list = np.asarray(cog_list).squeeze()
        com_pos.append(cog_list[0, :2].reshape((2, 1)))  # nex com target
        zmp_error.append(cog_list[0, 2:].reshape((2, 1)))

        # Buffer FIFO
        zmp_buffer = zmp_buffer[1:]

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
    clip = len(com_pos)

    print(len(com_pos))

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
