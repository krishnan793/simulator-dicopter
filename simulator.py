import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation
import math as math
import numpy as np
import time
from threading import Thread

fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')

#Writer = animation.writers['avconv']
#writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)

plt.ion()
line, = ax.plot([],[],[],'r-')

ax.set_xlim((-1,1))
ax.set_ylim((-1,1))
ax.set_zlim((-1,1))

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

Delta_t = 0.01       # Measurement interval

F_p = 20    # Propeller force in N
X_p = 0.1   # Distance from COG to proppeller in m
X_w = 0.1   # Distance from COG to wing in m
M = 0.1     # Mass in Kg
I = 1       # Momentum of inertia
g = 9.8     # Gravity in N
k_d = 0.5   # Drag coefficient

position = np.array([0.0,0.0,0.0])          # State (x,y,z)
orientation = np.array([0.01,0.0,0.0])      # Orientation (thetax, thetay, thetaz)
w = 0                                       # Angular velocity along z-axis

theta = 0;

def init():
    global line
    line.set_data([],[])
    line.set_3d_properties([])
    return line,


def dicopter_draw(f):
    global line,theta
    vl = [-1*X_w,0,0]
    vr = [X_w,0,0]
    axis = [0, 0, 1]
    vlnew=np.dot(rotation_matrix(axis,theta),vl)
    vrnew=np.dot(rotation_matrix(axis,theta),vr)

    line.set_data([vlnew[0],vrnew[0]],[vlnew[1],vrnew[1]])
    line.set_3d_properties([vlnew[2],vrnew[2]])
    return line,


def rotation_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    theta = np.asarray(theta)
    axis = axis/math.sqrt(np.dot(axis, axis))
    a = math.cos(theta/2.0)
    b, c, d = -axis*math.sin(theta/2.0)
    aa, bb, cc, dd = a*a, b*b, c*c, d*d
    bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
    return np.array(
        [[aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac)],
        [2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab)],
        [2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc]]
        )


def dicopterupdate(i):
    while (True):
        global theta,w,theta,Delta_t
        
        A = -1.0*k_d*w**2*X_p/I
        B =  1.0*F_p*X_p/I
        
        w_dot = A + B
        w = w + w_dot*Delta_t
        theta = (theta + w*Delta_t) % 360
        print w
        time.sleep(Delta_t)

thread1 = Thread( target=dicopterupdate, args=(0, ) )
thread1.start()

anim = animation.FuncAnimation(fig, dicopter_draw, init_func=init, frames=360, interval=1, blit=True, repeat=True)
#anim.save('in.mp4',writer=writer)
plt.show()
