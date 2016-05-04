import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation
import math as math
import numpy as np
import time
from threading import Thread
import joystick
import pygame

EXIT = 1

fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')

#Writer = animation.writers['avconv']
#writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)

control_theta = 999

plt.ion()
line, = ax.plot([],[],[],'b-')
line2, = ax.plot([],[],[],'r-')

ax.set_xlim((-0.1,0.1))
ax.set_ylim((-0.1,0.1))
ax.set_zlim((-0.1,0.1))

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

Delta_t = 0.001       # Measurement interval

F_p = 20    # Propeller force in N
X_p = 0.1   # Distance from COG to proppeller in m
X_w = 0.1   # Distance from COG to wing in m
M = 0.1     # Mass in Kg
I = 1       # Momentum of inertia
g = 9.8     # Gravity in N
k_dz = 0.01  # Drag coefficient along z axis
k_dy = 10    # Drag coefficient along x axis
k_l = 0.1   # Lift coefficient

Theta_l = 1     # Theta on left wing
Theta_r = 1     # Theta on right wing

position = np.array([0.0,0.0,0.0])          # State (x,y,z)
orientation = np.array([0.0,0.0,0.0])       # Orientation (theta_x, theta_y, theta_z)
velocity = np.array([0.0,0.0,0.0])          # Angular velocity (w_x, w_y, w_z)

vlnew = np.array([[-1.0*X_w],
                [0.0],
                [0.0],
                [1.0]])
vrnew = np.array([[1.0*X_w],
                [0.0],
                [0.0],
                [1.0]])
ztnew = np.array([[0.0],
                [0.0],
                [X_w],
                [1.0]])
zbnew = np.array([[0.0],

                [0.0],
                [0.0],
                [1.0]])

axis_x = [1, 0, 0]
axis_y = [0, 1, 0]
axis_z = [0, 0, 1]



def init():
    global line,line2
    line.set_data([],[])
    line.set_3d_properties([])
    line2.set_data([],[])
    line2.set_3d_properties([])

    return line,line2


def dicopter_draw(f):
    global line,line2,vlnew,vrnew,zline
    

    line.set_data([vlnew[0],vrnew[0]],[vlnew[1],vrnew[1]])
    line.set_3d_properties([vlnew[2],vrnew[2]])
    line2.set_data([zbnew[0],ztnew[0]],[zbnew[1],ztnew[1]])
    line2.set_3d_properties([zbnew[2],ztnew[2]])
    #ax.scatter(zline[0],zline[1],zline[2])
    return line,line2,

def transform_matrix(T_matrix,thetay,thetaz):
    """
    Input: Prev transform matrix (Identity Matrix if its the first time)
    output: A transform matrix which when multiplied with a point will give its new cordinate
    """
    return np.dot(T_matrix,np.dot(rotation_matrix('y',thetay),rotation_matrix('z',thetaz)))

def transform_point(T_matrix,X):
    """
    Return Transformed point.
    """
    return np.dot(T_matrix,X)


def translational_matrix(X):
    """
    Return the translational matrix that move point by X
    """
    return np.array(
        [[1,0,0,X[0]],
        [0,1,0,X[1]],
        [0,0,1,X[2]]]
        )
def rotation_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation
    about the given axis by theta radians.
    """
    if(axis == 'z'):
        return np.array(
            [[math.cos(theta),math.sin(theta),0,0],
            [-1*math.sin(theta),math.cos(theta),0,0],
            [0,0,1,0],
            [0,0,0,1]]
            )
    elif(axis == 'y'):
        return np.array(
            [[math.cos(theta),0,-1*math.sin(theta),0],
            [0,1,0,0],
            [math.sin(theta),0,math.cos(theta),0],
            [0,0,0,1]]
            )
    else:
        return 0


def dicopterupdate(i):
    global EXIT
    T_matrix = np.array(
        [[1,0,0,0],
        [0,1,0,0],
        [0,0,1,0],
        [0,0,0,1]]
        )
    while (EXIT):
        global orientation,w,Delta_t,theta,vlnew,vrnew,Theta_l,Theta_r,ztnew,zbnew,velocity,axis_x,axis_y,axis_z
        
        A = -1.0*k_dz*velocity[2]**2*X_p/I
        B =  1.0*F_p*X_p/I
        
        alpha_z = A + B
        velocity[2] = velocity[2] + alpha_z*Delta_t

        F_l = k_l*velocity[2]**2*Theta_l
        F_r = k_l*velocity[2]**2*Theta_r
        F = [0,0,F_l+F_r]
        
        alpha_y = (F_r-F_l)*X_w/I - 1.0*k_dy*velocity[1]**2*X_w/I
        #velocity[1] = velocity[1] + alpha_y*Delta_t

        orientation[2] = (orientation[2] + velocity[2]*Delta_t) % (2*math.pi)
        orientation[1] = (orientation[1] + velocity[1]*Delta_t) % (2*math.pi)

        #print velocity[1]

        vl = np.array([[-1*X_w],
                    [0],
                    [0],
                    [1]]
                    )
        vr = np.array([[X_w],
                    [0],
                    [0],
                    [1]]
                    )
        zt = np.array([[0],
                    [0],
                    [X_w],
                    [1]]
                    )
        zb = np.array([[0],
                    [0],
                    [0],
                    [1]]
                    )

        T_matrix = transform_matrix(T_matrix,velocity[1]*Delta_t,velocity[2]*Delta_t)

        #print velocity[1]*Delta_t
        vlnew = transform_point(T_matrix,vl)
        vrnew = transform_point(T_matrix,vr)
        ztnew = transform_point(T_matrix,zt)
        zbnew = transform_point(T_matrix,zb)
        #print vlnew
        #vlnew = np.dot(rotation_matrix(axis_z,velocity[2]*Delta_t),vlnew)
        #vrnew = np.dot(rotation_matrix(axis_z,velocity[2]*Delta_t),vrnew)
        #zline = np.dot(rotation_matrix(axis_z,velocity[2]*Delta_t),zline)

        #axis_x = np.dot(rotation_matrix(axis_z,velocity[2]*Delta_t),axis_x)     # Thetax
        #axis_y = np.dot(rotation_matrix(axis_z,velocity[1]*Delta_t),axis_y)     # Thetay

        #axis_z = np.dot(rotation_matrix(axis_y,velocity[1]*Delta_t),axis_y)

        #vlnew = np.dot(rotation_matrix(axis_y,velocity[1]*Delta_t),vlnew)     # Thetax
        #vrnew = np.dot(rotation_matrix(axis_y,velocity[1]*Delta_t),vrnew)     # Thetax
        #zline = np.dot(rotation_matrix(axis_y,velocity[1]*Delta_t),zline)

        #axis_z = np.dot(rotation_matrix(axis_y,velocity[1]*Delta_t),axis_y)

        #print 'Angular speed:',velocity[1]*Delta_t
        #print zline
        #print F_r,F_l
        time.sleep(Delta_t)

def temp_control():
    global control_theta,velocity
    while(True):
        control_theta = math.pi/4;
        time.sleep(1)
        #break;
        control_theta = 0;
        time.sleep(1)
        #control_theta = math.pi/2;
        #time.sleep(2)
        #control_theta = math.pi/8;
        #time.sleep(3)
                                
def control():
    global orientation,Theta_r,Theta_l,velocity,control_theta,EXIT
    i=0
    while(EXIT):
        #i += 1
        #print control_theta
        if (control_theta == 999):
            Theta_r = 1
            Theta_l = 1
            continue
        abs_theta = abs(orientation[2]-control_theta)
        #print Theta_r
        #Theta_r = abs_theta * (2 / math.pi) + 1
        #print Theta_r
        #time.sleep(0.01)
        #Theta_r = math.cos(abs_theta)*1.5 + 1
        #Theta_l = math.cos(abs_theta+math.pi/8)*0.5 + 1
        velocity[1] = math.cos(abs_theta)*1.5
        #Theta_r = math.cos(abs_theta)*1.5 + 1
        #Theta_l = math.cos(abs_theta+(math.pi/4))*1.5 + 1
        #print abs_theta
        #if(abs_theta < 0.5):
        #    velocity[1] = 1.5
        #    #Theta_l = 1
        #    #cos(abs_theta)*1.5
        #    #velocity[1] = cos(abs_theta)*1.5
        #    #pass
        #else:
        #    #pass
        #    #Theta_r = 1
        #    velocity[1] = 0
        #    #Theta_l = (abs_theta+math.pi) * (2 / math.pi) + 1
        #    #velocity[1] = 0
        time.sleep(0.001) 


def joystick_init():
    joy=[]
    pygame.joystick.init()
    pygame.display.init()
    if not pygame.joystick.get_count():
        print ("\nPlease connect a joystick and run again.\n")
        quit()
    print ("\n%d joystick(s) detected." % pygame.joystick.get_count())
    for i in range(pygame.joystick.get_count()):
        myjoy = pygame.joystick.Joystick(i)
        myjoy.init()
        joy.append(myjoy)
        print ("Joystick %d: " % (i) + joy[i].get_name())
    print ("Depress trigger (button 0) to quit.\n")
    # run joystick listener loop

joy_pos = [0.0,0.0]

def joystickControl():
    print "Hai"
    global joy_pos,control_theta,EXIT
    while (EXIT):
        e = pygame.event.wait()
        if (e.type == pygame.JOYAXISMOTION):
            key = joystick.handleJoyEvent(e)
            #print key['axis']
            if(key['axis'] == 2):
                joy_pos[0] = round(key['value'],2)
            if(key['axis'] == 3):
                joy_pos[1] = round(key['value'],2)
            if(joy_pos[0] == 0 and joy_pos[1] == 0):
                control_theta = 999
            else:
                t=math.atan2(joy_pos[1],joy_pos[0])
                if(t >= 0):
                    control_theta = t
                else:
                    control_theta = t+2*math.pi
            print joy_pos
        elif (e.type == pygame.JOYBUTTONDOWN):
            try:
                key = joystick.handleJoyEvent(e)
                print key['button']
                if(key['button'] == 0):
                    EXIT = 0
                    thread1.join()
                    thread2.join()
            except:
                pass
				
thread1 = Thread( target = dicopterupdate, args=(0, ) )
thread1.start()

thread2 = Thread( target = control, args=() )
thread2.start()

joystick_init()

thread3 = Thread( target = joystickControl, args=() )
thread3.start()

anim = animation.FuncAnimation(fig, dicopter_draw, init_func=init, frames=360, interval=1, blit=True, repeat=True)
#anim.save('in.mp4',writer=writer)
plt.show()

thread3.join()

