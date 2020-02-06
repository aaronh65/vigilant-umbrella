import numpy as np
import math
import time
from matplotlib import pyplot as plt
from scipy.stats import norm
import pdb

from MapReader import MapReader

class SensorModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 6.3]
    """

    def __init__(self, occupancy_map):

        """
        TODO : Initialize Sensor Model parameters here
        """
        # initialize all the parameters for the sensor model except the mean (comes from the raycasted val)
        # w1 w2 w3 w4 sigma lambda range
        self.w1 = 0.8
        self.w2 = 0.175
        self.w3 = 0.02
        self.w4 = 0.005
        self.sigma = 20
        self.lmbda = 0.001
        self.map = occupancy_map
        self.range = 8190
        self.norm = norm

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

    # calculates the probability of your measurement given sensor model pdf based on the raycasted mean val
    def getProbability(self, ray, meas):
        #gauss = 1/np.sqrt(2*np.pi*self.sigma**2) * np.exp(-(meas-ray)**2/(2*self.sigma**2))
        #gauss /= 
        gauss = self.norm.pdf(meas, loc=ray, scale=self.sigma)
        gauss_norm = self.norm.cdf(self.range, loc=ray, scale=self.sigma) - self.norm.cdf(0, loc=ray, scale=self.sigma)
        gauss = gauss / gauss_norm / self.sigma
        exp = 0
        if meas < ray:
            exp = self.lmbda*np.exp(-self.lmbda*meas)
            exp *= 1/(1-np.exp(-self.lmbda*ray))
        p_max = 0
        if np.abs(meas - self.range) <= 1:
            p_max = 1
        p_rand = 0
        if (meas > 0 and meas < self.range):
            p_rand = 1/self.range
        p = self.w1*gauss + self.w2*exp + self.w3*p_max + self.w4*p_rand
        if p == 1:
            print('PROBABILITY OVER ONE')
        #p = gauss
        return p

    def beam_range_finder_model(self, z_t1_arr, x_t1):
        """
        param[in] z_t1_arr : laser range readings [array of 180 values] at time t
        param[in] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        param[out] prob_zt1 : likelihood of a range scan zt1 at time t
        """

        """
        TODO : Add your code here
        """
        q=0
        # picking every other measurement for now

        x, y, theta = x_t1
        laser_origin = (x + np.cos(theta)*25, y + np.sin(theta)*25, theta)
        print(laser_origin)
        start_angle = theta - np.pi/2
        angles = [start_angle + np.pi/180 * n for n in range(180)]
        xqs, yqs = list(), list()
        for idx, angle in enumerate(angles):
            # calculate ray cast for the particle @ that angle
            # create probability distribution
            # calculate likelihood for that laser reading
            z_cast, xq, yq = self.range_find(laser_origin, angle)
            xqs.append(xq)
            yqs.append(yq)
            meas = z_t1_arr[idx]
            prob = self.getProbability(z_cast, meas)
            #print('z_cast = {}, meas = {}, prob = {}'.format(z_cast,meas,prob))
            q += np.log(prob)
        return q, xqs, yqs   

    def visualize(self):
        measurements = np.arange(0,self.range,1) + 1
        cast = np.arange(0, self.range,1)
        probs = self.getProbability(cast, measurements)
        plt.plot(measurements, dist)
        plt.show()

    def range_find(self, laser_origin, angle):
        x, y, theta = laser_origin
        sample_rate = 10
        z_cast = 0
        xq = np.round(x/10).astype(int)
        yq = np.round(y/10).astype(int)
        while self.map[yq,xq] == 0 and z_cast < self.range:
            dx = z_cast * np.cos(angle)
            dy = z_cast * np.sin(angle)
            xq = np.round((x + dx)/10).astype(int)
            yq = np.round((y + dy)/10).astype(int)
            z_cast += sample_rate
        return z_cast, xq*10, yq*10

if __name__=='__main__':
    model = SensorModel(None)
    model.visualize()
