import sys
import matplotlib.pyplot as plt
import numpy as np
import math
from utils import get_occupancy_map

class MotionModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 5]
    """

    def __init__(self):

        """
        TODO : Initialize Motion Model parameters here (e.g. alpha1, 2, 3, and 4, user-defined)
        """
        
        self.a1 = 0.001
        self.a2 = 0.001
        self.a3 = 0.001
        self.a4 = 0.001

    def wrap(self, angle):
        """
        param[in] angle : angle value in range [-inf, inf]
        param[out] angle_wrapped " angle value in range [-pi, pi]
        """
        
        """
        TODO : Write wrapper to keep angle sums/differences in range [-pi, pi]
        """
        
        # https://stackoverflow.com/questions/27093704/converge-values-to-range-pi-pi-in-matlab-not-using-wraptopi
        
        angle_wrapped = angle - 2*np.pi * np.floor((angle + np.pi) / (2*np.pi))
        
        return angle_wrapped

    def _sample(self, var):
        return np.random.normal(loc=0.0, scale=var)

    def update(self, u_t0, u_t1, x_t0):
        """
        param[in] u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]   
        param[in] u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
        param[in] x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
        param[out] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        """

        """
        TODO : Add your code here (algorithm from table 5.6 in thrun)
        """

        if u_t1[0] == u_t0[0] and u_t1[1] == u_t0[1] and u_t1[2] == u_t0[2]:
            #print('no motion')
            x_t1 = x_t0
            return x_t1

        rot1 = np.arctan2(u_t1[1] - u_t0[1], u_t1[0] - u_t0[0]) - u_t0[2]
        trans = np.sqrt((u_t1[0]-u_t0[0])**2 + (u_t1[1]- u_t0[1])**2)
        rot2 = u_t1[2] - u_t0[2] - rot1

        var_rot1 = self.a1*rot1**2 + self.a2*trans**2
        var_trans = self.a3*trans**2 + self.a4*rot1**2 + self.a4*rot2**2
        var_rot2 = self.a1*rot2**2 + self.a2*trans**2

        rot1_bar = rot1 - self._sample(var_rot1)
        trans_bar = trans - self._sample(var_trans)
        rot2_bar = rot2 - self._sample(var_rot2)

        x_t1 = np.zeros((3))
        x_t1[0] = x_t0[0] + trans_bar*np.cos(x_t0[2] + rot1_bar)
        x_t1[1] = x_t0[1] + trans_bar*np.sin(x_t0[2] + rot1_bar)
        x_t1[2] = x_t0[2] + rot1_bar + rot2_bar

        return x_t1.transpose()
        '''

        if u_t1[0] == u_t0[0] and u_t1[1] == u_t0[1] and u_t1[2] == u_t0[2]:
            #print('no motion')
            x_t1 = x_t0
            return x_t1

        xb, yb, thb = u_t0
        xbp, ybp, thbp = u_t1
        
        dr1 = np.arctan2(ybp - yb, xbp - xb) - thb
        dr1 = self.wrap(dr1)
        dt = np.sqrt(np.power(xb - xbp, 2) + np.power(yb - ybp, 2))
        dr2 = thbp - thb - dr1
        dr2 = self.wrap(dr2)
        
        dhr1 = dr1 - np.random.normal(scale=self.a1 * dr1**2 + self.a2*dt**2)
        dhr1 = self.wrap(dhr1)
        dht = dt - np.random.normal(scale=self.a3 * dt**2 + self.a4 * dr1**2 + self.a4 * dr2**2)
        dhr2 = dr2 - np.random.normal(scale=self.a1 * dr2**2 + self.a2 * dt**2)
        dhr2 = self.wrap(dhr2)
        
        x, y, th = x_t0
        
        xp = x + dht * np.cos(th + dhr1)
        #print(x, dht, th+dhr1)
        yp = y + dht * np.sin(th + dhr1)
        #print(y, dht, th+dhr1)
        thp = th + dhr1 + dhr2
        #print(th, dhr1+dhr2)
        thp = self.wrap(thp)
        
        x_t1 = np.array([xp, yp, thp])
        #x_t1 = np.reshape(x_t1, (3,1))
        return x_t1
        '''


if __name__=="__main__":
    occupancy_map = get_occupancy_map()
    src_path_log = '../data/log/robotdata1.log'
    logfile = open(src_path_log, 'r')
    motion_model = MotionModel()
    num_particles = 1
    y0_vals = np.random.uniform(4000,4010, (num_particles, 1))
    x0_vals = np.random.uniform(4150,4160, (num_particles, 1))
    theta0_vals = np.random.uniform(0*np.pi-0.01,0*np.pi+0.01, (num_particles, 1))
    X_bar = np.hstack((x0_vals, y0_vals, theta0_vals)).transpose()
    print('Initial particle position: {}'.format(X_bar))
    first_time_flag=True
    u_t0 = None;
    u_t1 = None
    xt, yt = list(), list()
    refX, refY = list(), list()
    for time_idx, line in enumerate(logfile):
        meas_vals = np.fromstring(line[2:], dtype=np.float64, sep=' ')
        odometry_robot = meas_vals[0:3]
        if (first_time_flag):
            u_t0 = odometry_robot
            first_time_flag = False
            continue
        X_bar_new = np.zeros((num_particles,3), dtype=np.float64)
        u_t1 = odometry_robot
        for m in range(0, num_particles):
            X_bar_new = motion_model.update(u_t0, u_t1, X_bar)
        X_bar = X_bar_new
        u_t0 = u_t1

        print('Step index: {}, robot @ {}'.format(str(time_idx), X_bar))
        xt.append(X_bar[0])
        yt.append(X_bar[1])
        refX.append(u_t0[0])
        refY.append(u_t1[1])
    plt.subplot(1,2,1)
    plt.plot(xt,yt)
    plt.scatter(xt[0],yt[0],c='g')
    plt.legend('robot')
    plt.subplot(1,2,2)
    plt.plot(refX, refY)
    plt.legend('ref')
    plt.show()
