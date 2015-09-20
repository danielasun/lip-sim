#!/usr/bin/env python

# Test to see how well the walk algorithm for this robot works.

import engine

kineState = (5, 1, 5, -1)  # x, vx, y, vy
footPlacement = (5, 5)


# choose style of walking.
walkstyle = "b"

s_x = []
s_y = []

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
    s_x = [0,  0,  0,  0,  0,  0,  0, 0]
    s_y = [0, .2, .2, .4, .2, .4, .2, 0]

stepOffsets = zip(s_x, s_y)
Robot = engine.WalkEngine(kineState, footPlacement, stepOffsets)

for i in range(len(s_x)-1):  # don't take the last step because we don't have a future step.
    Robot.walk()

x, vx, y, vy, p_x, p_y = Robot.output(display_on=True)
print x[-1], vx[-1], y[-1], vy[-1]
