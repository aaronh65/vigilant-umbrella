import sys
import numpy as np
import math

class MotionModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 5]
    """

    def __init__(self):

        """
        TODO : Initialize Motion Model parameters here (e.g. alpha1, 2, 3, and 4, user-defined)
        """
        
        self.a1 = 0.1#2
        self.a2 = 0.1#2
        self.a3 = 0.1#2
        self.a4 = 0.1#2

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
        
        x_t1 = [xp, yp, thp]

        return x_t1

if __name__=="__main__":
    mm = MotionModel()
    #print(mm.update([0, 0, 0], [1, 2, np.pi/2], [0, 0, 0]))
    print(mm.update([-1, -2, np.pi/2], [1, 2, np.pi/2], [0, 0, 0]))
    #print(mm.update([-1, -2, np.pi/4], [1, 2, np.pi/2], [0, 0, 0]))
    print(mm.update([-1, -2, 0], [1, 2, 0], [0, 0, 0]))
