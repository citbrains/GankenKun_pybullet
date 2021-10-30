#!/usr/bin/env python3
#
# preview control

import math
import numpy as np
import scipy
import scipy.linalg

import control
import control.matlab

import matplotlib.pyplot as plt
from copy import deepcopy

from foot_step_planner_v2 import *


class preview_control():
    """Walk pattern tracking using Preview Control method

    Parameters
    ----------
    dt : float, optional
        Sample time, by default 0.01
    preview_t : int, optional
        Future preview time (s), by default 2
    z : float, optional
        CoM height, by default 0.23
    R : float, optional
        Penalty on controls, by default 1e-6
    Qe : int, optional
        Penalty on ZMP error, by default 1
    Qdpos : int, optional
        Penalty on differential CoM position, by default 0
    Qdvel : int, optional
        Penalty on differential CoM velocity, by default 0
    Qdaccel : int, optional
        Penalty on differential CoM acceleration, by default 0
    debug : bool, optional
        Debug function, by default False
    """

    def __init__(self,
                 dt=0.01,
                 preview_t=2,
                 z=0.23,
                 R=1e-6,
                 Qe=1,
                 Qdpos=0,
                 Qdvel=0,
                 Qdaccel=0,
                 debug=False):

        self.debug = debug
        self.dt = dt
        self.n_preview = int(preview_t // self.dt)
        self.com_height = z
        G = 9.8
        A = np.matrix([
            [1.0, self.dt, self.dt**2 / 2],
            [0.0, 1.0, self.dt],
            [0.0, 0.0, 1.0]])
        B = np.matrix([self.dt**3 / 6, self.dt**2 / 2, self.dt]
                      ).reshape((3, 1))

        C = np.matrix([1, 0, -z / G]).reshape((1, 3))

        AA = np.vstack((np.hstack((np.eye(1), C * A)),
                        np.hstack((np.zeros((3, 1)), A))))

        BB = np.vstack((C * B, B))
        RR = R
        QQ = np.diag([Qe, Qdpos, Qdvel, Qdaccel])
        PP = scipy.linalg.solve_discrete_are(AA, BB, QQ, RR)

        # (R + b.T * P * b)^-1 (only the first value)
        SS = 1.0 / (RR + BB.T * PP * BB)[0, 0]
        KK = SS * BB.T * PP * AA
        Ke = KK[0, 0]
        Kx = KK[0, 1:4]

        # Design of an optimal controller for a discrete-time system
        # subject to previewable demand
        # P.680 (Theorem 1)

        Ac = AA - BB * KK
        XX = -Ac.T * PP * np.matrix([[1, 0, 0, 0]]).T * QQ[0, 0]

        G = np.zeros(self.n_preview)
        G[0] = -Ke

        for i in range(1, self.n_preview):
            G[i] = (SS * BB.T * XX)[0, 0]
            XX = Ac.T * XX

        Ks = Ke

        if (self.debug):
            print("T = ", dt)
            print("AA = \n", AA)
            print("BB = \n", BB)
            print("QQ = \n", QQ)
            print("RR = \n", RR)
            print("SS = \n", SS)
            print("KK = \n", KK)
            print("XX=\n", XX)
            print("Ks=", Ks)
            print("Kx=", Kx)
            print("G(start) =", G[:4])
            print("G(end) = ", G[-4:])
            print("G.sum() = ", G.sum())
            print("G size = ", len(G))

        self.A = A
        self.B = B
        self.C = C
        self.G = G
        self.Ks = Ks
        self.Kx = Kx

    def initStateErr(self, pos=0, vel=0, accel=0, e=0):
        """Initialize a stateErr object which holds position,
        velocity, acceleration, as a vector, and accumulated ZMP error
        as a scalar."""
        X = np.matrix([[pos, vel, accel]]).T

        return (X, e)

    def updateStateErr(self, stateErr, zmpref):
        """Run the controller. The stateErr argument should be a
        (state, error) tuple (as returned by initStateErr() or this
        function). The zmpref argument should be an array of future
        desired ZMP positions. If zmpref is of less length than the
        lookahead window size, the reference trajectory is padded with
        repeats of the last element.

        This function returns three values: the new stateErr after the
        control is executed, the new ZMP position, and the generated
        control.

        Reference: Design of an optimal controller for a discrete-time
        system subject to previewable demand
        """

        # extract future zmp trajectory
        zmpref = np.array(zmpref).flatten()
        nref = len(zmpref)
        if nref < self.n_preview:
            npad = self.n_preview - nref
            zrng = np.hstack((zmpref[0:], np.ones(npad) * zmpref[-1]))
        else:
            zrng = zmpref[0:self.n_preview]

        # get state
        X, e = stateErr

        # get control
        u = -self.Ks * e - self.Kx * X - np.dot(self.G, zrng)

        # update state & compute ZMP
        Xnew = self.A * X + self.B * u
        zmp = self.C * Xnew
        enew = e + zmp - zmpref[0]  # discrete integral of tracking error (24)

        # return new state, ZMP, and control
        return (Xnew, enew), zmp, u

    def update_preview_controller(self, state_x, state_y, zmpref):
        """Calculate the feature state based on the current state and reference ZMP
        """
        COG_X = []

        zmp_x = 0
        zmp_y = 0
        for i in range(len(zmpref)):
            state_x, zmp_x, _ = self.updateStateErr(
                state_x, zmpref[i:, 0])
            state_y, zmp_y, _ = self.updateStateErr(
                state_y, zmpref[i:, 1])

            COG_X += [np.block([state_x[0][0], state_y[0][0], zmp_x, zmp_y])]

        return COG_X, state_x, state_y


def vc_step_planner():

    preview_t = 1.5
    pc_dt = 0.0015
    sys_dt = 0.001

    planner = foot_step_planner(
        dt=sys_dt, n_steps=4, dsp_ratio=0.0, t_step=0.34)
    pc = preview_control(dt=pc_dt, preview_t=preview_t)

    fig, axs = plt.subplots(2, 2)
    fig.tight_layout(pad=1.0)
    axs = axs.ravel()

    # Set current state as Xk-1
    state_x = pc.initStateErr(pos=0, e=0)
    state_y = pc.initStateErr(pos=0, e=0)

    supp_foot = np.asarray([0, -0.03525, 0]).reshape((3, 1))
    torso = np.asarray([0, 0.0, 0]).reshape((3, 1))
    # support right with next in left
    foot_step, torso_pos, zmp_pos, timer_count = planner.calculate(
        (0.2, 0.0, 0.0), supp_foot, torso, 'left', True)

    foot_step0 = np.asarray(foot_step)
    zmp_pos = np.asarray(zmp_pos)
    torso_pos = np.asarray(torso_pos)

    axs[0].plot(foot_step0[:, 0].astype(np.float32),
                foot_step0[:, 1].astype(np.float32), 'o-', label="Foot X Reference")
    axs[1].plot(foot_step0[:, 0].astype(np.float32),
                foot_step0[:, 2].astype(np.float32), 'o-', label="Foot Y Reference")
    axs[0].plot(torso_pos[:, 0].astype(np.float32),
                torso_pos[:, 1].astype(np.float32), 'go-', label="Torso X Reference")
    axs[1].plot(torso_pos[:, 0].astype(np.float32),
                torso_pos[:, 2].astype(np.float32), 'go-', label="Torso Y Reference")
    axs[0].set_title("Foot & Torso - X Reference")
    axs[0].legend(loc='upper left', prop={'size': 10})
    axs[0].set_ylabel('X (meters)')
    axs[1].set_title("Foot & Torso - Y Reference")
    axs[1].legend(loc='upper left', prop={'size': 10})
    axs[1].set_ylabel('Y (meters)')

    cog_list = []

    cog_list, _, _ = pc.update_preview_controller(
        state_x, state_y, zmp_pos)

    cog_list = np.asarray(cog_list).squeeze()

    axs[2].plot(cog_list[:, 0], label='COM X')
    axs[2].plot(cog_list[:, 2], label='ZMP X Prediction')
    axs[2].plot(zmp_pos[:, 0], label='ZMP X Reference')
    axs[2].legend(loc='upper left', prop={'size': 10})
    axs[2].set_title('ZMP preview controller - inputs & outputs')
    axs[2].set_ylabel('X (meters)')

    axs[3].plot(cog_list[:, 1], label='COM Y')
    axs[3].plot(cog_list[:, 3], label='ZMP Y Prediction')
    axs[3].plot(zmp_pos[:, 1], label='ZMP Y Reference')
    axs[3].legend(loc='upper left', prop={'size': 10})
    axs[3].set_title('ZMP preview controller - inputs & outputs')
    axs[3].set_ylabel('Y (meters)')

    [s.set_xlabel("time t(s)") for s in axs[-2:]]

    plt.show()


if __name__ == '__main__':
    vc_step_planner()
    # vc_step_planner_v2()
