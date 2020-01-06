"""
Inverse Kinematic controller
"""

import numpy as np
import sys

class Controller(object):
    """ developes controls solutions """


    def __init__(self):
        # time step
        self.dt = 0.1

        # acceleration limits
        self.v_dot_lim = 0.288
        self.w_dot_lim = 5.579

        # control perturbation parameters
        self.std_w = 5*0.0175 # 1 deg = 0.0175 rad
        self.std_v = 0.01 # 1cm/s

        # noise in pose parameters
        self.std_x = 0.02 # 2cm
        self.std_y = 0.02# 2cm
        self.std_theta = 0.0175*5 # 5 deg

        # FD calculation
        self.prev_v = None

        # store previous controls
        self.u_v_prev = 0
        self.u_w_prev = 0

        # gains
        self.kw = 2
        self.kv = 0.5

        #self.kw = 5
        #self.kv = 1


    def update_pose(self, pose, u):
        """ updates pose given controls """

        pose_new = self.rk4(self.kinematic_model, pose, u)

        # wrap -pi to pi
        if pose_new[2] > np.pi:
            pose_new[2] -= 2*np.pi
        elif pose_new[2] < -np.pi:
            pose_new[2] += 2*np.pi

        # add noise to pose
        #pose[0] += np.random.normal(0.02, self.std_x**2)
        #pose[1] += np.random.normal(0.02, self.std_y**2)
        #pose[2] += np.random.normal(0.0175/2, self.std_theta**2)
        pose[0] = np.random.normal(0, self.std_x**2)
        pose[1] += np.random.normal(0, self.std_y**2)
        pose[2] += np.random.normal(0, self.std_theta**2)

        return pose_new


    def finite_diff(self, curr_v):
        """ finite difference """

        v_dot = (curr_v - self.prev_v) / self.dt
        return v_dot

    def record_velocities(self, u):
        """ record current accelerations """

        self.prev_v = u


    def get_accelerations(self, u):
        """ compute current accelerations using FD """

        # current velocities are the control
        curr_v = np.array([u[0], u[1]])

        # finite diff
        v_dot = self.finite_diff(curr_v)

        return v_dot

    def controls(self, pose, waypoint):
        """ updates the conrols given

        Args:
            pose (np.array): x, y, theta of robot
            waypoint (np.array): x and y coordinates of waypoint

        Returns:
            u (np.array): linear and angular velocity controls
        """

        # position of waypoint
        xg = waypoint[0]
        yg = waypoint[1]

        # robot pose
        xr = pose[0]
        yr = pose[1]
        theta_r = pose[2]

        # bearing from robot to waypoint
        b = np.arctan2(yg - yr, xg - xr)

        # error in heading
        h_error = b - theta_r

        # wrap heading error -pi t0 pi to turn efficiently
        if h_error > np.pi:
            h_error -= 2*np.pi
        elif h_error < -np.pi:
            h_error += 2*np.pi

        # error in position
        d_error = np.sqrt((xg-xr)**2 + (yg-yr)**2)

        # nominal control update
        u_w = self.kw*h_error
        u_v = self.kv*d_error

        # perturb controls
        u_w += np.random.normal(0, self.std_w**2)
        u_v += np.random.normal(0, self.std_v**2)

        # get accelerations based on control updates
        accels = self.get_accelerations([u_v, u_w])


        # update controls based on acceleration limits
        # check directions of acceleration
        if abs(round(accels[0], 3)) > self.v_dot_lim:
            if round(accels[0], 3) < 0:
                u_v = self.u_v_prev - self.v_dot_lim*self.dt + self.std_v

            elif round(accels[0], 3) > 0:
                u_v = self.u_v_prev + self.v_dot_lim*self.dt - self.std_v

        if abs(round(accels[1], 3)) > self.w_dot_lim:
            if round(accels[1], 3) < 0:
                u_w = self.u_w_prev - self.w_dot_lim*self.dt + self.std_w

            elif round(accels[1], 3) > 0:
                u_w = self.u_w_prev + self.w_dot_lim*self.dt - self.std_w


        """
        accels = self.get_accelerations([u_v, u_w])

        if round(accels[0],3) > self.v_dot_lim:
            print("linear acceleration too large")
            print("v_dot: ", accels[0])

        if round(accels[1],3) > self.w_dot_lim:
            print("angular acceleration too large")
            print("w_dot: ", accels[1])
            print("-----------------")
        """

        # make prev
        self.record_velocities([u_v, u_w])

        # store current controls
        self.u_v_prev = u_v
        self.u_w_prev = u_w

        return np.array([u_v, u_w])


    def kinematic_model(self, pose, u):
        """ Kinematic unicycle model

        Args:
            pose (np.array): state vector containing robot x, y, and theta
            u (np.array): linear and angular velocity controls

        Returns:
            x_dot (np.array): x_dot, y_dot, theta_dot
        """
        v = u[0]
        w = u[1]

        x = pose[0]
        y = pose[1]
        theta = pose[2]

        x_dot = np.array([v*np.cos(theta),
                          v*np.sin(theta),
                          w])

        return x_dot


    def rk4(self, f, x, u):
        """ performs one iteration of rk4 method

        Args:
            f (function handle): model to integrate
            x (np.array): state vector containing robot x, y, and theta
            u (np.array): linear and angular controls

        Returns:
            x_new (np.array): updated pose
        """

        k1 = f(x, u) * self.dt
        k2 = f(x + k1/2, u) * self.dt
        k3 = f(x + k2/2, u) * self.dt
        k4 = f(x + k3, u) * self.dt
        x_new = x + 1/6*(k1 + 2*k2 + 2*k3 + k4)
        return x_new















#
