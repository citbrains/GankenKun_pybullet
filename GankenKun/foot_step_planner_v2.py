import math
import numpy as np


class foot_step_planner():
    """Foot step planner for N consecutive steps

    Parameters
    ----------
    n_steps : int, optional
        Number of steps to generate, by default 6
    t_step : float, optional
        Time (s) for a single step, by default 0.25
    dsp_ration : float, optional
        The ratio for double support phase, by default 0.15
    dt : float, optional
        Sample time, by default 0.01
    """

    def __init__(
            self,
            max_stride_x=0.05,
            max_stride_y=0.03,
            max_stride_th=0.2,
            n_steps=4,
            foot_separation=0.03525,
            t_step=0.25,
            dsp_ratio=0.15,
            dt=0.01):

        # Time Param
        self.t_step = t_step
        self.dsp_ratio = dsp_ratio
        self.t_dsp = self.dsp_ratio * self.t_step
        self.dt = dt  # integral and counter
        self.t_dsp_1 = (self.t_dsp / 2)
        self.t_dsp_2 = (self.t_dsp / 2)

        self.t_begin = (self.t_dsp / 2)
        self.t_end = self.t_step - (self.t_dsp / 2)
        self.norm_t_begin = self.t_begin / self.t_step
        self.norm_t_end = self.t_end / self.t_step

        # Walk Parameters
        assert n_steps >= 3, 'Error: Number of steps must be > 3'
        self.n_steps = n_steps
        self.y_sep = foot_separation  # Trunk to leg sep

        # TODO: Limit the step length for T
        self.max_stride_x = max_stride_x
        self.max_stride_y = max_stride_y
        self.max_stride_th = max_stride_th

    def mod_angle(self, a):
        """Mod angle to keep in [-pi, pi]

        Parameters
        ----------
        a : float
            Input angle (rad)

        Returns
        -------
        float
            Mod/clamped angle in [-pi, pi]
        """

        a = a % (2 * np.pi)
        if (a >= np.pi):
            a = a - 2 * np.pi
        return a

    def calcHfunc(self, t_time, norm_t):
        """Horizontal Trajectory function

        Reference: Maximo, Marcos - Omnidirectional ZMP-Based Walking for Humanoid
        Section 3.3 - CoM Trajectory Using 3D-LIPM

        Calculate the trajectory using C1-continuous spline interpolation eq. (3.36)

        Parameters
        ----------
        t_time : float
            The time since the beginning of the step
        norm_t : float
            Normalized time since the beginning of the step [0, 1]

        Returns
        -------
        float
            Value of interpolation at time t_time
        """
        h_func = 0
        if norm_t < self.norm_t_begin:
            h_func = 0
        elif norm_t >= self.norm_t_begin and norm_t < self.norm_t_end:
            h_func = 0.5 * \
                (1 - np.cos(np.pi * ((t_time - self.t_begin) / (self.t_end - self.t_begin))))
        elif norm_t >= self.norm_t_end:
            h_func = 1
        return h_func

    def calcVfunc(self, t_time, norm_t):
        """Vertical Trajectory function

        Reference: Maximo, Marcos - Omnidirectional ZMP-Based Walking for Humanoid
        Section 3.3 - CoM Trajectory Using 3D-LIPM

        Calculate the trajectory using C1-continuous spline interpolation eq. (3.39)

        Parameters
        ----------
        t_time : float
            The time since the beginning of the step
        norm_t : float
            Normalized time since the beginning of the step [0, 1]

        Returns
        -------
        float
            Value of interpolation at time t
        """

        v_func = 0
        if norm_t < self.norm_t_begin or norm_t >= self.norm_t_end:
            v_func = 0
        elif norm_t >= self.norm_t_begin and norm_t < self.norm_t_end:
            v_func = 0.5 * (1 - np.cos(2 * np.pi * ((norm_t -
                            self.norm_t_begin) / (self.norm_t_end - self.norm_t_begin))))
        return v_func

    def pose_global2d(self, prelative, gpose):
        """Calculate the 2d pose of prelative w.r.t gpose

        A (Global), B (Local)

        Calculate pose B w.r.t to frame A

        Parameters
        ----------
        prelative : np.ndarray
            Target pose to be rotated (x,y,theta)
        gpose : np.ndarray
            The reference frame in 2D (x,y,theta)

        Returns
        -------
        np.ndarray
            Rotated prelative in gpose frame (x',y',theta')
        """

        prelative = np.asarray(prelative, dtype=np.float32).reshape((3, 1))
        gpose = np.asarray(gpose, dtype=np.float32).reshape((3, 1))

        assert prelative.shape == (3, 1), 'Shape must be (3,1)'
        assert gpose.shape == (3, 1), 'Shape must be (3,1)'

        ca = np.cos(gpose[2])
        sa = np.sin(gpose[2])

        return np.array([gpose[0] + ca * prelative[0] - sa * prelative[1],
                        gpose[1] + sa * prelative[0] + ca * prelative[1],
                        gpose[2] + prelative[2]])

    def pose_relative2d(self, pglobal, prelative):
        """Calculate the 2d pose of pglobal w.r.t prelative

        In simple terms, the inverse of pose_global2d function
        A (Global), B (Local)

        Calculate pose A w.r.t to frame B

        Parameters
        ----------
        pglobal : np.ndarray
            Target pose to be rotated (x,y,theta)
        prelative : np.ndarray
            The reference frame in 2D (x,y,theta)

        Returns
        -------
        np.ndarray
            Rotated gpose w.r.t prelative frame (x',y',theta')
        """

        pglobal = np.asarray(pglobal, dtype=np.float32).reshape((3, 1))
        prelative = np.asarray(prelative, dtype=np.float32).reshape((3, 1))

        assert pglobal.shape == (3, 1), 'Shape must be (3,1)'
        assert prelative.shape == (3, 1), 'Shape must be (3,1)'

        ca = np.cos(prelative[2])
        sa = np.sin(prelative[2])

        px = pglobal[0] - prelative[0]
        py = pglobal[1] - prelative[1]
        pa = pglobal[2] - prelative[2]
        return np.array([ca * px + sa * py, -sa * px + ca * py, self.mod_angle(pa)])

    def calcNextPose(self, cmd_vel, init_pose):
        """Calculate the next COM pose from the current input

        Reference: Maximo, Marcos - Omnidirectional ZMP-Based Walking for Humanoid
        Section 3.2 - Selecting Next Torso and Swing Foot Poses

        Calculate the next COM (in this case the torso) pose based on the current
        input velocity using nonlinear differential equations (3.5). The current
        implementation use equation (3.6) by integrating equation (3.5) for a step duration T. The discrete
        time model of equation (3.7) and (3.8) is not used.

        Parameters
        ----------
        init_pose : np.ndarray
            Current torso pose in 2D (x,y,theta)

        Returns
        -------
        np.ndarray
            The expected 2D pose after a step duration T (x',y',theta')
        """

        v_x = 0
        v_y = 0
        v_z = 0

        cmd_x, cmd_y, cmd_a = cmd_vel

        new_pose = init_pose.copy()

        steps = int(self.t_step // self.dt)
        for _ in range(steps):
            v_x += (cmd_x * np.cos(new_pose[2] + cmd_a * self.dt) -
                    cmd_y * np.sin(new_pose[2] + cmd_a * self.dt)) * self.dt
            v_y += (cmd_x * np.sin(new_pose[2] + cmd_a * self.dt) +
                    cmd_y * np.cos(new_pose[2] + cmd_a * self.dt)) * self.dt
            v_z += cmd_a * self.dt

        new_pose[0] += v_x
        new_pose[1] += v_y
        new_pose[2] += v_z
        return new_pose

    def calcTorsoFoot(self, vel_cmd, current_supp, current_torso, left_is_swing):
        """Calculate the target torso and swing foot

        Reference: Maximo, Marcos - Omnidirectional ZMP-Based Walking for Humanoid
        Section 3.2 - Selecting Next Torso and Swing Foot Poses

        Calculate the next torso pose T[k+1] and A[k+1] from the current T[k] and S[k]
        Read page 54 - page 59 and equations from (3.9) to (3.21)
        The reference frame is in the Support[k] foot
        """
        # w.r.t Support[k] globally

        if isinstance(current_supp, list) or isinstance(current_supp, tuple):
            current_supp = np.asarray(
                current_supp, dtype=np.float32).reshape(3, 1)

        if isinstance(current_torso, list) or isinstance(current_torso, tuple):
            current_torso = np.asarray(
                current_torso, dtype=np.float32).reshape(3, 1)

        if isinstance(vel_cmd, list):
            vel_cmd = np.asarray(vel_cmd)

        cmd_x, cmd_y, cmd_a = vel_cmd
        init_supp_pos = current_supp
        init_torso_pos = current_torso

        # target torso w.r.t S[K] using eq. (3.6)
        torso_k1 = self.calcNextPose(vel_cmd, init_torso_pos)

        # Check leg is open or close eq. (3.11)
        o_y = 1 if ((cmd_y >= 0 and left_is_swing) or
                    (cmd_y < 0 and not left_is_swing)) else 0

        # Check leg is open or close eq. (3.12)
        o_z = 1 if ((cmd_a >= 0 and left_is_swing) or
                    (cmd_a < 0 and not left_is_swing)) else 0

        # TODO: Check safety algorithm
        # Z Safety check to make sure support no pointing inward > 15 deg
        a_diff = (torso_k1[2] - init_supp_pos[2])
        if np.abs(a_diff) > np.radians(15):
            torso_k1[2] -= (np.radians(15) - a_diff)

        # Y torso safety check maintain foot and torso >= y_sep
        y_diff = (torso_k1[1] - init_supp_pos[1])
        if np.abs(y_diff) < self.y_sep:
            diff_ = self.pose_global2d(np.array([[0], [np.abs(y_diff)], [
                0]], dtype=np.float32), np.array([[0], [0], [torso_k1[2]]], dtype=float))
            if left_is_swing:
                torso_k1[1] += diff_[1]
            else:
                torso_k1[1] -= diff_[1]

        # w.r.t Torso[K+1]
        torso_k2 = self.calcNextPose(vel_cmd, torso_k1)

        # Decoupled case using eq. (3.13)
        if o_z:
            P_frame = torso_k2
        else:
            P_frame = torso_k1

        # Calculate the relative pose of T[k+1] and T[k+2] w.r.t P Frame
        rel_p_xt_1 = self.pose_relative2d(torso_k1, P_frame)
        rel_p_xt_2 = self.pose_relative2d(torso_k2, P_frame)

        # Calculate the swing foot Theta direction eq. (3.14) to align frame
        swing_foot = np.zeros((3, 1))
        foot_pose_a = self.pose_relative2d(
            np.array([[0], [0], [0]], dtype=np.float32), P_frame)
        swing_foot[2] = foot_pose_a[2]

        # Calculate the swing foot pose in Y w.r.t P frame using eq. (3.15)
        if o_y:
            foot_pose_y = rel_p_xt_2[1] + \
                (np.power((-1), 1 - left_is_swing) * self.y_sep)
        else:
            foot_pose_y = rel_p_xt_1[1] + \
                (np.power((-1), 1 - left_is_swing) * self.y_sep)
        swing_foot[1] = foot_pose_y

        # Calculate the half difference in X from T[k+1] to T[k+2] w.r.t P Frame
        # using eq. (3.16) to eq. (3.19)
        if o_z:
            angle_rot = rel_p_xt_1[2] + cmd_a * self.t_step
            diff_x = (rel_p_xt_2[0] - rel_p_xt_1[0]) * np.cos(angle_rot) - \
                (rel_p_xt_2[1] - rel_p_xt_1[1]) * np.sin(angle_rot)
        else:
            angle_rot = rel_p_xt_1[2]
            diff_x = (rel_p_xt_2[0] - rel_p_xt_1[0]) * np.cos(angle_rot) - \
                (rel_p_xt_2[1] - rel_p_xt_1[1]) * np.sin(angle_rot)

        # Calculate the swing foot in X w.r.t P Frame using eq. (3.22)
        foot_pose_x = rel_p_xt_1[0] + (diff_x / 2)
        swing_foot[0] = foot_pose_x

        # Save the result of target torso T[k+1] and swing foot position w.r.t global frame S[k]
        target_swing_pos = self.pose_global2d(
            swing_foot, P_frame)  # align with global frame
        target_swing_pos[2] = P_frame[2]  # realign angle
        target_torso_pos = torso_k1

        return target_torso_pos, target_swing_pos

    def compute_zmp_trajectory(self, zmp_t, init_torso_2d, target_torso_2d, init_supp_2d):
        """Compute the zmp trajectory as a piecewise linear function.

        Reference: Yi, Seung-Joon - Whole-Body Balancing Walk Controller
        for Position Controlled Humanoid Robots
        Section 3.1 - Footstep generation controller

        Compute the zmp trajectory from the initial torso position
        to the target torso position through the support foot.

        Parameters
        ----------
        zmp_t : [type]
            Current zmp time
        init_torso_2d : [type]
            Initial torso position (x,y)
        target_torso_2d : [type]
            Target torso position (x,y)
        init_supp_2d : [type]
            Initial support leg position (x,y)

        Returns
        -------
        zmp_pos: List
            List of computed zmp trajectory
        timer_count: List
            List of zmp timer count
        """

        t_time = 0
        zmp_pos = []
        timer_count = []

        init_torso_2d = np.asarray(
            init_torso_2d, dtype=np.float32)
        target_torso_2d = np.asarray(
            target_torso_2d, dtype=np.float32)
        init_supp_2d = np.asarray(
            init_supp_2d, dtype=np.float32)

        m_dsp1 = (init_supp_2d - init_torso_2d) / self.t_dsp_1

        while (t_time < self.t_step):
            if t_time < self.t_begin:
                x_zmp_2d = init_torso_2d + m_dsp1 * t_time
            elif t_time >= self.t_begin and t_time < self.t_end:
                x_zmp_2d = init_supp_2d
            elif t_time >= self.t_end:
                x_zmp_2d = target_torso_2d + \
                    ((init_supp_2d - target_torso_2d) / self.t_dsp_2) * \
                    (self.t_step - t_time)  # From SJ. YI
            timer_count.append(zmp_t)
            zmp_pos.append(x_zmp_2d.ravel())
            t_time += self.dt
            zmp_t += self.dt

        return zmp_pos, timer_count

    def calculate(self, vel_cmd, current_supp, current_torso, next_support_leg, sway=True):
        """Calculate N consecutive steps from the given parameters

        Parameters
        ----------
        vel_cmd : Tuple
            Velocity command input
        current_supp : np.ndarray
            Current support foot position
        current_torso : np.ndarray
            Current torso position
        next_support_leg : np.ndarray
            Next support leg sequence
        phase : str
            Current walking phase between double support phase (dsp)
            or single support phase (ssp), 
            by default double support phase (dsp) 

        Returns
        -------
        foot_step: List
            List of computed foot steps position
        torso_pos: List
            List of computed torso position
        zmp_pos: List
            List of zmp trajectory position
        timer_count: List
            List of zmp timer count
        """

        time = 0.0

        # first step
        torso_pos = []
        foot_step = []
        zmp_pos = []
        zmp_t = 0
        timer_count = []
        left_is_swing = 1

        if isinstance(current_supp, np.ndarray):
            current_supp = np.squeeze(current_supp).tolist()
        if isinstance(current_torso, np.ndarray):
            current_torso = np.squeeze(current_torso).tolist()

        # Add the initial state
        foot_step += [[time, current_supp[0],
                       current_supp[1], current_supp[2], 'both']]

        torso_pos += [[time, current_torso[0],
                       current_torso[1], current_torso[2]]]
        time += self.t_step

        # Add reference to reduce initial error
        if sway:
            for i in range(int(self.t_step // self.dt)):
                zmp_pos.append(np.asarray(current_torso[:2], dtype=np.float32))
                zmp_t += self.dt
                timer_count.append(zmp_t)

        # Compute the next state
        if next_support_leg == 'right':
            left_is_swing = 0
            target_torso_pos, target_swing_pos = self.calcTorsoFoot(
                vel_cmd, current_supp, current_torso, left_is_swing)

            torso_pos += [[time, *target_torso_pos.ravel()]]
            foot_step += [[time, *target_swing_pos.ravel(), next_support_leg]]

            current_torso = target_torso_pos
            current_supp = target_swing_pos

            next_support_leg = 'left'
            left_is_swing = 1

        elif next_support_leg == 'left':
            target_torso_pos, target_swing_pos = self.calcTorsoFoot(
                vel_cmd, current_supp, current_torso, left_is_swing)

            torso_pos += [[time, *target_torso_pos.ravel()]]
            foot_step += [[time, *target_swing_pos.ravel(), next_support_leg]]

            current_torso = target_torso_pos
            current_supp = target_swing_pos

            next_support_leg = 'right'
            left_is_swing = 0

        temp_zmp, temp_tzmp = self.compute_zmp_trajectory(
            zmp_t, torso_pos[-2][1:3], torso_pos[-1][1:3], foot_step[-2][1:3])
        zmp_pos += temp_zmp
        timer_count += temp_tzmp

        # Compute the steps for N future steps
        for i in range(self.n_steps - 2):
            target_torso_pos, target_swing_pos = self.calcTorsoFoot(
                vel_cmd, current_supp, current_torso, left_is_swing)
            time += self.t_step
            torso_pos += [[time, *target_torso_pos.ravel()]]
            foot_step += [[time, *target_swing_pos.ravel(), next_support_leg]]

            if left_is_swing:
                next_support_leg = 'right'
                left_is_swing = 0
            else:
                next_support_leg = 'left'
                left_is_swing = 1

            current_torso = target_torso_pos
            current_supp = target_swing_pos

            temp_zmp, temp_tzmp = self.compute_zmp_trajectory(
                zmp_t, torso_pos[-2][1:3], torso_pos[-1][1:3], foot_step[-2][1:3])
            zmp_pos += temp_zmp
            timer_count += temp_tzmp

        # Add the 2 last step return to center
        # if not status == 'stop':
        time += self.t_step

        if next_support_leg == 'left':
            final_support_pos = self.pose_global2d(np.array(
                [[0], [2 * self.y_sep], [0]], dtype=np.float32), current_supp)  # align with support foot
            target_torso_pos = self.pose_global2d(
                np.array([[0], [self.y_sep], [0]], dtype=np.float32), current_supp)
        else:
            final_support_pos = self.pose_global2d(np.array(
                [[0], [-2 * self.y_sep], [0]], dtype=np.float32), current_supp)  # align with support foot
            target_torso_pos = self.pose_global2d(
                np.array([[0], [-self.y_sep], [0]], dtype=np.float32), current_supp)

        torso_pos += [[time, *target_torso_pos.ravel()]]
        foot_step += [[time, *final_support_pos.ravel(), next_support_leg]]

        temp_zmp, temp_tzmp = self.compute_zmp_trajectory(
            zmp_t, torso_pos[-2][1:3], torso_pos[-1][1:3], foot_step[-2][1:3])
        zmp_pos += temp_zmp
        timer_count += temp_tzmp

        # Following the idea to hack pc
        time += self.t_step
        next_support_leg = 'both'
        foot_step += [[time, *final_support_pos.ravel(), next_support_leg]]
        torso_pos += [[time, *target_torso_pos.ravel()]]
        for i in range(int(self.t_end // self.dt)):
            zmp_pos.append(target_torso_pos[:2].ravel())
            zmp_t += self.dt
            timer_count.append(zmp_t)

        return foot_step, torso_pos, zmp_pos, timer_count


if __name__ == '__main__':
    planner = foot_step_planner()
    foot_step, torso_pos, zmp_po, timer_count = planner.calculate(
        (0.0, 0, 0), (0, -0.03525, 0), (0, 0, 0), 'right', 'start')
    for i in foot_step:
        print(i)
