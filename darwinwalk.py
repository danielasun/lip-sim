#!/usr/bin/env python

# Test for LIP walking on Darwin OP

import engine

kineState = (0, 0, 0, 0)  # x, vx, y, vy
footPlacement = (0, 0)


# choose style of walking.
walkstyle = "b"

s_x = []
s_y = []

dx = .07
dy = .05

if walkstyle == "b": # basic walking
    s_x = [0, 0, dx, dx, dx, dx, 0]
    s_y = [0, dy, dy, dy, dy, dy, 0]
if walkstyle == "diag":
    s_x = [0,  0, .02, .02, .02,  0, 0]
    s_y = [0, .02, .03, .01, .03, .02, 0]
if walkstyle == "my":  # minimal side to side movement
    s_x = [0, .2, .2, .2, .2, .2, .2, 0]
    s_y = [0,  0,  0,  0,  0,  0,  0, 0]
if walkstyle == "sw":  # sidewalking
    s_x = [0,  0,  0,  0,  0,  0,  0, 0]
    s_y = [0, dy, dy, 2*dy, dy, 2*dy, dy, 0]

stepOffsets = zip(s_x, s_y)
Robot = engine.WalkEngine(kineState, footPlacement, stepOffsets)
Robot.zHeight = .3
Robot.Tsup = .9
Robot.weightA = 10
Robot.weightB = .5

for i in range(len(s_x)-1):  # don't take the last step because we don't have a future step.
    Robot.walk()

x, vx, y, vy, p_x, p_y = Robot.output(display_on=True)
print x[-1], vx[-1], y[-1], vy[-1]
