#!/usr/bin/env python

# Test to see how well the walk algorithm for this robot works.

import engine

kineState = (0, 0, 0.08, 0)  # x, vx, y, vy
footPlacement = (0, 0)

# choose style of walking.
walkstyle = "jeff"

s_x = []; s_y = []
dx = .05; dy = .6

if walkstyle == "b": # basic walking
    s_x = [0, .0, .3, .3, .3,  0, 0]
    s_y = [0, .2, .2, .2, .2, .2, 0]
if walkstyle == "diag":
    s_x = [0,  0, .2, .2, .2,  0, 0]
    s_y = [0, .2, .3, .1, .3, .2, 0]
if walkstyle == "my":  # minimal side to side movement
    s_x = [0, .2, .2, .2, .2, .2, .2, 0]
    s_y = [0,  0,  0,  0,  0,  0,  0, 0]
if walkstyle == "sw":  # sidewalking
    s_x = [0, 0,  0,   0,   0,   0,  0,   0,  0,  0]
    s_y = [0, .2, -.2, .2, -.2, .2, -.2, .2, -.2,  0]
if walkstyle == "long": # walk for a long time
    s_x.append(0); s_y.append(0)
    s_x.append(0); s_y.append(dy)
    for step in range(8):
        s_x.append(dx)
        s_y.append(dy)
    s_x.append(0); s_y.append(0)
if walkstyle == "jeff":
    s_x.append(0); s_y.append(0)
    for step in range(8):
        s_x.append(dx)
        s_y.append(.6)
        s_x.append(dx)
        s_y.append(.4)
    s_x.append(0); s_y.append(0)

stepOffsets = zip(s_x, s_y)
print stepOffsets

Robot = engine.WalkEngine(kineState, footPlacement, stepOffsets)
Robot.zHeight = .5
Robot.Tsup = 0.32
Robot.weightA = 10
Robot.weightB = 1

for i in range(len(s_x)-1):  # don't take the last step because we don't have a future step.
    Robot.walk()

x, vx, y, vy, p_x, p_y = Robot.output(display_on=True)
print x[-1], vx[-1], y[-1], vy[-1]
