#!/usr/bin/env python3
# 
# preview control

import math
import numpy as np
import control
import control.matlab

class preview_control():
  def __init__(self, dt, period, z, Q = 1.0e+8, H = 1.0):
    self.dt = dt
    self.period = period
    G = 9.8
    A = np.matrix([
      [0.0, 1.0, 0.0],
      [0.0, 0.0, 1.0],
      [0.0, 0.0, 0.0]])
    B = np.matrix([[0.0], [0.0], [1.0]])
    C = np.matrix([[1.0, 0.0, -z/G]])
    D = 0
    sys = control.matlab.ss(A, B, C, D)
    sys_d = control.c2d(sys, dt)
    self.A_d, self.B_d, self.C_d, D_d = control.matlab.ssdata(sys_d)
    E_d = np.matrix([[dt], [1.0], [0.0]])
    Zero = np.matrix([[0.0], [0.0], [0.0]])
    Phai = np.block([[1.0, -self.C_d * self.A_d], [Zero, self.A_d]])
    G = np.block([[-self.C_d*self.B_d], [self.B_d]])
    GR = np.block([[1.0], [Zero]])
    Gd = np.block([[-self.C_d*E_d], [E_d]])
    Qm = np.zeros((4,4))
    Qm[0][0] = Q
    P = control.dare(Phai, G, Qm, H)[0]
    self.F = -np.linalg.inv(H+G.transpose()*P*G)*G.transpose()*P*Phai
    xi = (np.eye(4)-G*np.linalg.inv(H+G.transpose()*P*G)*G.transpose()*P)*Phai;
    self.f = []
    for i in range(1,int(period/dt)):
      self.f += [-np.linalg.inv(H+G.transpose()*P*G)*G.transpose()*np.linalg.matrix_power(xi.transpose(),i-1)*P*GR]

  def set_param(self, pos, vel, acc, foot_plan):
    x, y = np.matrix([[pos[0]],[vel[0]],[acc[0]]]), np.matrix([[pos[1]],[vel[1]],[acc[1]]])
    xp, yp = x.copy(), y.copy()
    ux, uy = 0.0, 0.0
    COG_X = []
    for i in range(int(foot_plan[1][0]/self.dt)+1):
      px, py = self.C_d * x, self.C_d * y
      ex, ey = foot_plan[0][1] - px, foot_plan[0][2] - py
      X, Y = np.block([[ex], [x - xp]]), np.block([[ey], [y - yp]])
      xp, yp = x.copy(), y.copy()
      dux, duy = self.F * X, self.F * Y
      index = 1
      for j in range(1,int(self.period/self.dt)-1):
        if self.dt * (i + j) >= foot_plan[index][0]:
          dux += self.f[j] * (foot_plan[index][1]-foot_plan[index-1][1])
          duy += self.f[j] * (foot_plan[index][2]-foot_plan[index-1][2])
          index += 1
      ux, uy = ux + dux, uy + duy
      x, y = self.A_d * x + self.B_d * ux, self.A_d * y + self.B_d * uy
      COG_X += [np.block([x[0][0], y[0][0]])]
    return COG_X

if __name__ == '__main__':
  pc = preview_control(0.01, 1.0, 0.27)
  foot_step = [[0, 0, 0], [0.6, 0.1, 0.06], [0.9, 0.2, -0.06], [1.2, 0.2, 0.0], [3.2, 0.2, 0.0]]
  print(pc.set_param([0,0], [0,0], [0,0], foot_step))
