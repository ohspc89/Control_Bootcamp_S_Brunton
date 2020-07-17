'''
sim_cartpend.py is a rough conversion of the m-file sim_cartpend.m
(https://github.com/bertozzijr/Control_Bootcamp_S_Brunton/blob/master/sim_cartpend.m)

I have written this code while learning Steve Brunton's [Control Bootcamp].
(https://www.youtube.com/watch?v=qjhAAQexzLg&list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m&index=12)

Few things to note:
    1. I've included the two m-files in this script: drawcartpend_bw.m & cartpend.m
    2. For MATLAB's ode45, I've used solve_ivp from the scipy
    3. I tried to also use python_drawnow (https://github.com/stsievert/python-drawnow)
       so as to make the code look similar to the original m-file,
       but my lacking coding skill prevented me from using the module.
       However, I've still used the algorithm of drawnow - using plt.clf and plotting iteratively -
       and the [figure] class.
    4. Any modification / improvement is welcome!
'''

import numpy as np
from control import ctrb, place
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from drawnow import drawnow, figure

# This is the python version of drawcartpend_bw.m
def drawcartpend(y, m, M, L):
    x = y[0]
    th = y[2]

    W = 1*np.sqrt(M/5)
    H = .5*np.sqrt(M/5)
    wr = .2
    mr = .3*np.sqrt(m)

    y = wr/2 + H/2
    w1x = x - .9*W/2
    w1y = 0
    w2x = x + .9*W/2 - wr
    w2y = 0

    px = x + L*np.sin(th)
    py = y - L*np.cos(th)

    fig = plt.gcf()
    ax = fig.add_subplot(111)
    ax.plot([-10, 10], [0,0], 'w', linewidth = 2)
    ax.add_patch(Rectangle((x-W/2, y-H/2), W, H, fc='red', ec = 'red'))
    ax.add_patch(Circle((w1x, w1y), wr, fc='white', ec='white'))
    ax.add_patch(Circle((w2x, w2y), wr, fc='white', ec='white'))

    ax.plot([x, px], [y, py], 'w', linewidth = 2)
    ax.add_patch(Rectangle((px-mr/2, py-mr/2), mr, mr, fc='yellow', ec='yellow'))

    ax.set_xlim([-5, 5])
    ax.set_ylim([-2, 2.5])
    ax.set_facecolor('black')
    plt.show()

# This is the python version of cartpend.m
def cartpend(t, y, m, M, L, g, d, u):

    Sy = np.sin(y[2])
    Cy = np.cos(y[2])
    D = m*L*L*(M+m*(1-Cy ** 2))

    dy = [0] * 4
    dy[0] = y[1]
    dy[1] = (1/D)*(-m**2 * L**2 * g * Cy * Sy + m*L**2*(m*L*y[3]**2*Sy - d*y[1])) + m*L*L*(1/D)*u
    dy[2] = y[3]
    dy[3] = (1/D)*((m+M)*m*g*L*Sy - m*L*Cy*(m*L*y[3]**2*Sy - d*y[1])) - m*L*Cy*(1/D)*u + .01*np.random.randn()

    return dy

# From here, sim_cartpend.m
m = 1
M = 5
L = 2
g = -10
d = 1

tspan = np.arange(0, 10, 0.1)
y0 = [0, 0, np.pi, .5]

# t0 of [0, 15] is an arbitrary one. 
# It can be any tuple whose width is greater than range(10)
sol = solve_ivp(cartpend, [0, 15], y0, t_eval = tspan, args = (m, M, L, g, d, 0))

y = sol.y.T

# Iteratively drawing to make it look like an animation
figure()
for i in range(100):
    drawcartpend(y[i,], m, M, L)
    plt.pause(0.01)
    plt.clf() 
