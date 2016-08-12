# Gourab Datta
# 227071
import numpy as np
import pylab as pl

from math import *
import random

landmarks  = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
world_size = 100.0

# Creating the class robot
class robot:
    def __init__(self):
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi
        self.forward_noise = 0.0;
        self.turn_noise    = 0.0;
        self.sense_noise   = 0.0;
    
    def set(self, new_x, new_y, new_orientation):
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)
    
    
    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        # makes it possible to change the noise parameters
        self.forward_noise = float(new_f_noise);
        self.turn_noise    = float(new_t_noise);
        self.sense_noise   = float(new_s_noise);
    
    
    def sense(self):
        Z = []
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            Z.append(dist)
        return Z
    
    
    def move(self, turn, forward):
        if forward < 0:
            raise ValueError, 'Robot cant move backwards'         
        
        # turn, and add randomness to the turning command
        orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
        orientation %= 2 * pi
        
        # move, and add randomness to the motion command
        dist = float(forward) + random.gauss(0.0, self.forward_noise)
        x = self.x + (cos(orientation) * dist)
        y = self.y + (sin(orientation) * dist)
#        x %= world_size    # cyclic truncate
#        y %= world_size
        
        # set particle
        res = robot()
        res.set(x, y, orientation)
        res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
        return res
    
    def Gaussian(self, mu, sigma, x):
        
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))
    
    
    def measurement_prob(self, measurement):
        
        # calculates how likely a measurement should be
        
        prob = 1.0;
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            prob *= self.Gaussian(dist, self.sense_noise, measurement[i])
        return prob
      
    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))



def eval(r, p):
    sum = 0.0;
    for i in range(len(p)): # calculate mean error
        dx = (p[i].x - r.x + (world_size/2.0)) % world_size - (world_size/2.0)
        dy = (p[i].y - r.y + (world_size/2.0)) % world_size - (world_size/2.0)
        err = sqrt(dx * dx + dy * dy)
        sum += err
    return sum / float(len(p)) # Average deviation of robot position from particles
    
def get_position(p):
    x = 0.0
    y = 0.0
    orientation = 0.0
    for i in range(len(p)):
        x += p[i].x
        y += p[i].y
        orientation += (((p[i].orientation - p[0].orientation + pi) % (2.0 * pi)) 
                        + p[0].orientation - pi)
    return [x / len(p), y / len(p), orientation / len(p)]


myrobot = robot()
N = 500
T = 50

# Creating Particles
p = []
for i in range(N):
    r = robot()
    r.set_noise(0.05, 0.05, 5.0)
    p.append(r)

myrobotx,myroboty = [],[]
avg_err = []
px = [[[0]*N for j in range(T)]for k in range (2)]
est_pos = [[0]*3 for j in range(T)]

for t in range(T):
    # Device true movement    
    myrobot = myrobot.move(0.1, 5.0)
    myrobotx.append(myrobot.x)
    myroboty.append(myrobot.y)
    Z = myrobot.sense()

    # Particel movement
    p2 = []
    for i in range(N):
        p2.append(p[i].move(0.1, 5.0))
    p = p2

    # Assigning weights to particle
    w = []
    for i in range(N):
        w.append(p[i].measurement_prob(Z))

    p3 = []
    index = int(random.random() * N)
    beta = 0.0
    mw = max(w)
    # Resampling particles based on weight
    for i in range(N):
        beta += random.random() * 2.0 * mw
        while beta > w[index]:
            beta -= w[index]
            index = (index + 1) % N
        p3.append(p[index])
        px[0][t][i] = (p3[i].x)
        px[1][t][i] = (p3[i].y)        
    p = p3
    
    # Estimating position from particles    
    est_pos[t] = get_position(p)
    # Estimating position error compared to true positions    
    avg_err.append(eval(myrobot, p))
    print avg_err[t]

# Plotting particles, true positions and estimated positions
pl.close('all')
pl.figure(1)
pl.hold(True)
for t in range(T):
    if t%2 > 0:
        pl.plot(px[0][t][0:499],px[1][t][0:499],'ro')
    else:
        pl.plot(px[0][t][0:499],px[1][t][0:499],'go')
pl.plot(myrobotx,myroboty,'bo-')
est_pos1 = array(est_pos)
pl.plot(est_pos1[:,0],est_pos1[:,1],'yo-')
pl.hold(False) 
pl.title('True Path in Blue, estimated path in yellow')
t1 = np.arange(T)
pl.figure(2)
pl.plot(t1,avg_err)
pl.title('Average error growth with time')
