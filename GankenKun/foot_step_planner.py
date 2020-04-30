#!/usr/bin/env python3
# 
# foot step planner for the GankenKun

import math

class foot_step_planner():
  def __init__(self, dt, max_stride_x, max_stride_y, max_stride_th, period, width):
    self.dt = dt
    self.max_stride_x = max_stride_x
    self.max_stride_y = max_stride_y
    self.max_stride_th = max_stride_th
    self.period = period
    self.width = width

  def calculate(self, goal_x,goal_y, goal_th, next_support_leg, status):
    self.goal_x = goal_x
    self.goal_y = goal_y
    self.goal_th = goal_th
    self.next_support_leg = next_support_leg
    self.status = status
    self.time = 0.0

    # calculate the number of foot step
    goal_distance = math.sqrt(self.goal_x**2 + self.goal_y**2)
    step_x  = abs(self.goal_x )/self.max_stride_x
    step_y  = abs(self.goal_y )/self.max_stride_y
    step_th = abs(self.goal_th)/self.max_stride_th
    self.max_step = max(step_x, step_y, step_th)

    self.stride_x  = self.goal_x /self.max_step
    self.stride_y  = self.goal_y /self.max_step
    self.stride_th = self.goal_th/self.max_step

    foot_step = []

    # first step
    if status == 'start':
      foot_step += [[0.0, 0.0, 0.0, 0.0, 'both']]
      self.time += self.period * 2.0
    if self.next_support_leg == 'right':
      foot_step += [[self.time, 0.0, -self.width, 0.0, self.next_support_leg]]
      self.next_support_leg = 'left'
    elif self.next_support_leg == 'left':
      foot_step += [[self.time, 0.0,  self.width, 0.0, self.next_support_leg]]
      self.next_support_leg = 'right'

    # walking
    current_x = current_y = current_th = 0.0
    counter = 0
    while True:
      counter += 1
      diff_x  = abs(self.goal_x  - current_x )
      diff_y  = abs(self.goal_y  - current_y )
      diff_th = abs(self.goal_th - current_th)
      if diff_x <= self.max_stride_x and diff_y <= self.max_stride_y and diff_th <= self.max_stride_th:
        break
      next_x  = current_x  + self.stride_x
      next_y  = current_y  + self.stride_y
      next_th = current_th + self.stride_th
      self.time += self.period
      if self.next_support_leg == 'right':
        foot_step += [[self.time, next_x, next_y-self.width, next_th, self.next_support_leg]]
        self.next_support_leg = 'left'
      elif self.next_support_leg == 'left':
        foot_step += [[self.time, next_x, next_y+self.width, next_th, self.next_support_leg]]
        self.next_support_leg = 'right'
      current_x, current_y, current_the = next_x, next_y, next_th

    # last step
    next_x, next_y, next_th = self.goal_x, self.goal_y, self.goal_th
    if not status == 'stop':
      self.time += self.period
      if self.next_support_leg == 'right':
        foot_step += [[self.time, next_x, next_y-self.width, next_th, self.next_support_leg]]
      elif self.next_support_leg == 'left':
        foot_step += [[self.time, next_x, next_y+self.width, next_th, self.next_support_leg]]
      self.time += self.period
      self.next_support_leg = 'both'
      foot_step += [[self.time, next_x, next_y, next_th, self.next_support_leg]]
      self.time += 2.0
      foot_step += [[self.time, next_x, next_y, next_th, self.next_support_leg]]

    return foot_step

if __name__ == '__main__':
  planner = foot_step_planner(0.01, 0.06, 0.04, 0.1, 0.34, 0.044)
  foot_step = planner.calculate(0.3, 0.0, 0.0, 'right', 'start')
  print(foot_step)
