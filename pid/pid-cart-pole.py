import matplotlib.pyplot as plt
import numpy as np
import math

# theta conditions
theta_d = math.pi
theta = 0
theta_dot = 0
theta_dotdot = 0

# x conditions
x = 0
x_dot = 0
x_dotdot = 0

# error conditions
e = theta_d - theta
e_old = 0
e_tot = 0

# system params
m1 = 0.5
m2 = 0.2
m = m1+m2
g = 9.8
l = 0.3

# pid gain terms
kP=32
kD=600
kI=0.2


# lists to store different values
angular_pos = [0]
force = [0]
pos = [0]

t = np.arange(0,10,0.01)

for i in range(0,999):
    e = theta_d - theta
    e_dot = e - e_old
    e_tot += e

    #controller equation
    u = (kP*e) + (kD*e_dot) + (kI*e_tot) 
    if u>=-0.5 and u<=0.5:
        u = 0

    # using model dynamics to calculate acceleration
    x_dotdot = (u - m2*l*theta_dotdot*math.cos(theta) + m2*l*theta_dot*theta_dot*math.sin(theta))/m

    if u>=-0.5 and u<=0.5:
        x_dotdot = 0

    # using model dynamics to calculate angular acceleration
    theta_dotdot = (-g*math.sin(theta)-x_dotdot*math.cos(theta))/l

    x_dot += x_dotdot*0.01
    theta_dot += theta_dotdot*0.01
    
    if u>=-0.5 and u<=0.5:
        x_dot = 0

    # using NLM to calculate angle and position
    x += (0.5*x_dotdot*0.0001) + x_dot*0.01
    theta += (0.5*theta_dotdot*0.0001) + theta_dot*0.01

    
    angular_pos.append(theta)
    force.append(u)
    pos.append(x)
    print(theta)
    
    e_old = e


# function to animate the cart pole system
def plot():
    import math
    from matplotlib.animation import FuncAnimation
    from matplotlib import pyplot as plt
    from math import pi
    from matplotlib import rc
    rc('animation',html='jshtml')
    rod_length = 20

    fig = plt.figure(figsize=(8,6.4))
    ax = fig.add_subplot(111,autoscale_on=False,\
                        xlim=(-50,80),ylim=( -50,50))
    mass1, = ax.plot([],[],linestyle='None',marker='s',\
                        markersize=30,markeredgecolor='k',\
                        color='orange',markeredgewidth=2)
    mass2, = ax.plot([],[],linestyle='None',marker='o',\
                        markersize=20,markeredgecolor='k',\
                        color='red',markeredgewidth=2)
    line, = ax.plot([],[],'o-',color='orange',lw=4,\
                        markersize=6,markeredgecolor='k',\
                        markerfacecolor='k')
    def animate(i):
        x = pos[i]
        y = 0
        angle = -1*angular_pos[i] +  pi 
        
        mass1.set_data([pos[i]],[0])
        mass2.set_data([x + rod_length*math.sin(angle)],[y + rod_length*math.cos(angle) ])
        line.set_data([x ,x + rod_length*math.sin(angle) ],[y , y + rod_length*math.cos(angle)])
        return  mass1 , mass2 , line ,
    
    anim = FuncAnimation(fig, animate, init_func = None, 
                        frames = len(pos), interval = 100, blit = True ) 

    # anim.save('optimize.gif', writer='pillow', fps=60)
    plt.show()

plot()

#plots of theta, force and position

theta_plot = plt.figure(1)
plt.plot(t,angular_pos)
plt.xlabel('time')
plt.ylabel('theta')
plt.title('Theta Plot')
plt.hlines(y=3.14, xmin=0, xmax=10, linewidth=2, color='r')

force_plot = plt.figure(2)
plt.plot(t,force)
plt.xlabel('time')
plt.ylabel('force')
plt.title('Force Plot')
plt.hlines(y=0, xmin=0, xmax=10, linewidth=2, color='b')

pos_plot = plt.figure(3)
plt.plot(t,pos)
plt.xlabel('time')
plt.ylabel('position')
plt.title('Position Plot')

plt.show()     

