#!/usr/bin/env python

"""
Main file for running the LIP simulation. Based on algorithms described in
Introduction to Humanoid Robotics by Shuuji Kajita, et. al.
Daniel Sun, September 8, 2015
"""

from math import cosh, sinh, sqrt
import matplotlib.pyplot as plt


class WalkEngine:
    def __init__(self, kineState, footPlacement, stepOffsets):
        # non changable parameters.
        self.zHeight = .8
        self.Tsup = .8
        self.Tc = sqrt(self.zHeight/9.81)
        self.dt = .001
        self.weightA = 10
        self.weightB = 1
        self.step = 0  # Kajita starts at 1 because he was using Matlab
        self.time = []

        # user defined parameters.
        self.xi, self.xvi, self.yi, self.yvi = kineState
        self.px, self.py = footPlacement
        self.sx, self.sy = zip(*stepOffsets)

        # footplacement variables
        self.p_xlist = []
        self.p_ylist = []
        self.pstarx = []
        self.pstary = []

        # output vectors
        self.xout = []
        self.xvout = []
        self.yout = []
        self.yvout = []

    def walk(self):
        """
        This is the main walking algorithm. Call this function every time you want to take a step.
        It calculates where to place the feet so that the pendulum doesn't lose control. It also saves
        the result of solving the EOM in the output lists so that you can write out the results to your
        motors/actuators etc.


        # Test for the output:
        >>> import engine
        >>> kineState = (0, 0, 0, 0)  # x, vx, y, vy
        >>> footPlacement = (0, 0)
        >>> s_x = [0, .3, .3, .3, .3, .3, 0]
        >>> s_y = [.2, .2, .2, .2, .2, .2, .2]
        >>> stepOffsets = zip(s_x, s_y)
        >>> Robot = engine.WalkEngine(kineState, footPlacement, stepOffsets)
        >>> Robot.walk()
        >>> Robot.walk()
        >>> Robot.walk()
        >>> Robot.walk()
        >>> Robot.walk()
        >>> Robot.walk()
        >>> x, vx, y, vy, _, _  = Robot.output(display_on=False)
        Calculating output...
        >>> print x[-1], vx[-1], y[-1], vy[-1]
        1.49916897455 0.00234175498652 0.229792524926 -0.577113775217
        """

        # Renaming constants, just to make things a little less wordy.
        n = self.step
        Tc = self.Tc
        Tsup = self.Tsup
        z = self.zHeight

        # Calculate the next foot place. px and py from the previous get overwritten.
        self.px, self.py = calcfootplace(self.px, self.py, self.sx[n], self.sy[n], n)
        self.p_xlist.append(self.px)
        self.p_ylist.append(self.py)

        # Calculate the walk primitive.
        xbar, ybar, vxbar, vybar = calcwalkprimitive(self.sx[n+1], self.sx[n+1], Tc, Tsup, n)

        # Calculate the target state.
        xd = self.px + xbar
        yd = self.py + ybar

        # Append the pstar values to the list
        self.pstarx.append(calcmodifiedfootplacement(xd, vxbar, self.xi, self.xvi, Tsup, z, self.weightA, self.weightB))
        self.pstary.append(calcmodifiedfootplacement(yd, vybar, self.yi, self.yvi, Tsup, z, self.weightA, self.weightB))

        # Solve eom and write result to arrays.

        # Create time vector
        time = [self.dt*i for i in range(int(self.Tsup/self.dt))]

        # x(t), v_x(t)
        xresult, xvresult = zip(*[eomsolve(t, self.xi, self.xvi, self.pstarx[-1], self.zHeight) for t in time])
        # y(t), v_y(t)
        yresult, yvresult = zip(*[eomsolve(t, self.yi, self.yvi, self.pstary[-1], self.zHeight) for t in time])

        # add them to the output lists.
        self.xout.extend(xresult)
        self.xvout.extend(xvresult)
        self.yout.extend(yresult)
        self.yvout.extend(yvresult)

        # update state variables.
        self.xi = self.xout[-1]
        self.yi = self.yout[-1]
        self.xvi = self.xvout[-1]
        self.yvi = self.yvout[-1]

        # always increment step after walking.
        self.step += 1

    def output(self, display_on):
        if display_on:
            print "Displaying data..."
            # Create time vector
            time = [self.dt*i for i in range(len(self.xout))]

            assert(len(self.xout) == len(self.yout) and len(self.xvout) == len(self.yvout))

            plt.figure(1)
            plt.subplot(221)
            plt.plot(time, self.xout, 'b-')
            plt.subplot(222)
            plt.plot(time, self.yout, 'b-')
            plt.subplot(223)
            plt.plot(time, self.xvout, 'b-')
            plt.subplot(224)
            plt.plot(time, self.yvout, 'b-')

            plt.figure(2)
            plt.plot(self.xout, self.yout, 'b--', self.pstarx, self.pstary, 'rx', self.p_xlist, self.p_ylist, 'ko')

            plt.show()
        else:
            print "Calculating output..."
        return self.xout, self.xvout, self.yout, self.yvout, self.p_xlist, self.p_ylist


def eomsolve(t, pos, vel, pstar, z):
    """
    Solve the EoM for the timestep from t to t + tsup.
    Generates the equation of motion for the pendulum given
    the initial conditions and the desired footstep. Eq 4.53

    Unit Tests:
    No velocity or displacement generates no
    movement.
    >>> '%.6f, %.6f' % eomsolve(0,0,0,0,.8)
    '0.000000, 0.000000'

    No velocity or displacement relative to pstar also generates
    no movement.
    >>> '%.6f, %.6f' % eomsolve(0,.1,0,0.1,.8)
    '0.100000, 0.000000'

    Since there is no velocity at time = 0, there is still no change
    in position or velocity after 1 second.
    >>> '%.6f, %.6f' % eomsolve(100,.1,0,0.1,.8)
    '0.100000, 0.000000'

    Starting at one end of the walk primitive should result in the symmetric
    result after Tsup.
    >>> '%.4f, %.4f' %  eomsolve(.8,-.15,.593184,0,.8)
    '0.1500, 0.5932'

    Same thing forwards and backwards
    >>> '%.4f, %.4f' %  eomsolve(.8,.15,-.593184,0,.8)
    '-0.1500, -0.5932'

    Having that same difference between x and pstar should not result in anything different.
    >>> '%.4f, %.4f' %  eomsolve(.4,-.15+1,.593184,1,.8)
    '1.0000, 0.2756'

    Being at t = Tsup/2 should have the position be at approximately zero.
    >>> '%.4f, %.4f' %  eomsolve(.4,-.15,.593184,0,.8)
    '0.0000, 0.2756'
    """

    Tc = sqrt(z / 9.81)  # time constant

    # vector that has position and time for the next
    nextPos = (pos - pstar) * cosh(t / Tc) + Tc * vel * sinh(t / Tc) + pstar
    nextVel = (pos - pstar) / Tc * sinh(t / Tc) + vel * cosh(t / Tc)

    return nextPos, nextVel


def calcfootplace(p_x_last, p_y_last, s_x, s_y, step): # this equation is DONE
    """
    Simply calculates the next footstep given the previous footsteps
    and the step lengths for the current one. If the 0th step is the starting
    position then the first leg to step is the left one. Kajita's index started
    from 1, hence the difference in calculating p_y.

    Returns p_x, p_y for the current step.

    >>> calcfootplace(0,0,1,1,0)
    (1, 1)
    >>> calcfootplace(0,0,1,1,1)
    (1, -1)
    >>> calcfootplace(0,0,1,1,2)
    (1, 1)
    >>> calcfootplace(3,3,-1,2,2)
    (2, 5)

    """
    return p_x_last + s_x, p_y_last + ((-1)**step)*s_y


def calcwalkprimitive(s_x_f, s_y_f, Tc, Tsup, step):
    """
    Determines the walk primitive for the nth step based on the step
    offset for the n+1th step.
    Returns pos_bar (x,y) for the nth step.
    Eq 4.51 & 4.52

    Examples:

    No step offset means the walk primitives are zero.
    >>> '%.6f, %.6f, %.6f, %.6f' % calcwalkprimitive(0, 0, sqrt(.8/9.81), .8,1)
    '0.000000, 0.000000, 0.000000, 0.000000'

    Typical input values for large robots at size/scale of HRP:
    >>> '%.6f, %.6f, %.6f, %.6f' % calcwalkprimitive(.3, .2, sqrt(.8/9.81), .8,1)
    '0.150000, -0.100000, 0.593184, -0.310085'

    Notice that the sign of the y direction primitives changes when the step changes from 1 (in the previous example) to 2.
    >>> '%.6f, %.6f, %.6f, %.6f' % calcwalkprimitive(.3, .2, sqrt(.8/9.81), .8,2)
    '0.150000, 0.100000, 0.593184, 0.310085'

    Sign is the same on alternate steps.
    >>> '%.6f, %.6f, %.6f, %.6f' % calcwalkprimitive(.3, .2, sqrt(.8/9.81), .8,3)
    '0.150000, -0.100000, 0.593184, -0.310085'

    Can also use this to go backwards.
    >>> '%.6f, %.6f, %.6f, %.6f' % calcwalkprimitive(.1, -.1, sqrt(.8/9.81), .8,2)
    '0.050000, -0.050000, 0.197728, -0.155043'

    Typical values for the sidewalker.
    >>> '%.6f, %.6f, %.6f, %.6f' % calcwalkprimitive(0, .05, sqrt(.3/9.81), .8,2)
    '0.000000, 0.025000, 0.000000, 0.140042'
    """
    C = cosh(Tsup/Tc)
    S = sinh(Tsup/Tc)

    xbar = s_x_f/2.0
    ybar = (-1)**step*s_y_f/2.0
    vxbar = xbar*(C+1)/(Tc*S)
    vybar = ybar*(C-1)/(Tc*S)

    return xbar, ybar, vxbar, vybar

def calcmodifiedfootplacement(xd, vd, xi, vi, Tsup, z , a, b):
    """
    Adjusts the placement of the feet according to the evaluation function
    to make the end position of the CoM closer to the correct place.
    Even though the evaluation function N originally depended on the final
    foot placement, since we know an equation for the final foot placement in
    terms of the initial foot placement and p*, we only use x_i and p*.
    Eq 4.59

    Suggested weights: a = 10, b = 1


    Example:
    Say that we are at x = 0, v = .5. with the typical HRP settings for z height,
    support time, etc. We want to move to x = .2, v = .25 in the given support time
    but we can't do it with the regular "put the foot halfway between your destination."

    >>> '%.6f, %.6f' % eomsolve(.8, 0, .5, .1, .8)
    '0.444918, 1.259454'

    Instead, we minimize the cost function so that the error is minimum and use
    the improved foot placement.
    >>> '%.6f' % calcmodifiedfootplacement(.2, .25, .0, .5, .8, .8, 10, 1)
    '0.134583'

    Using the modified p_star = .134583, we place our foot out farther and slow ourselves
    down over a longer distance so that we can come close to the desired position and speed.
    >>> '%.6f, %.6f' % eomsolve(.8, 0, .5, .134583, .8)
    '0.193692, 0.265966'
    """

    Tc = sqrt(z/9.81)  # time constant
    C = cosh(Tsup/Tc); S = sinh(Tsup/Tc)  # sinh and cosh
    D = a*(C-1)**2 + b*(S/Tc)**2

    return -a*(C-1)/D*(xd - C*xi - Tc*S*vi) \
           -b*S/(Tc*D)*(vd - S/Tc*xi - C*vi)


# debugging class
def dbf(msg):  # create time vector
    _FLAGS_ON = True
    if _FLAGS_ON:
        print msg

if __name__ == "__main__":
    import doctest
    doctest.testmod()
