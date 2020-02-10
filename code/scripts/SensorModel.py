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

    def __init__(self, occupancy_map, lookup_flag=False):

        """
        TODO : Initialize Sensor Model parameters here
        """
        # initialize all the parameters for the sensor model except the mean (comes from the raycasted val)
        # w1 w2 w3 w4 sigma lambda range
        self.w1 = 100
        self.w2 = 2
        self.w3 = 5
        self.w4 = 500
        self.sigma = 50
        self.lmbda = 0.01
        self.map = occupancy_map
        self.range = 8190
        self.ray_step = 5
        if lookup_flag:
            self.lookup = np.load('lookup_zero.npy')
        else:
            self.lookup = None


    def visualize_dist(self):
        measurements = np.arange(0,self.range,1) + 1
        cast = np.arange(0, self.range,1)
        probs = self.getProbability(cast, measurements)
        plt.plot(measurements, dist)
        plt.show()


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
        # gauss
        if 0 < meas < self.range:
            gauss_norm = norm(loc=ray, scale=self.sigma).cdf(self.range) - norm(loc=ray, scale=self.sigma).cdf(0)
            gauss = norm(loc=ray, scale=self.sigma).pdf(meas) / gauss_norm
        else:
            gauss = 0

        # short
        if 0 < meas < ray:
            exp = self.lmbda*np.exp(-self.lmbda*meas)
            exp *= 1/(1-np.exp(-self.lmbda*ray))
        else:
            exp = 0

        # out of range
        if np.abs(meas - self.range) <= 1:
            p_max = 1
        else:
            p_max = 0

        # random
        if (meas > 0 and meas < self.range):
            p_rand = 1/self.range
        else:
            p_rand = 0
        p = self.w1*gauss + self.w2*exp + self.w3*p_max + self.w4*p_rand
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
        x, y, theta = x_t1
        laser_pose = (x + np.cos(theta)*25, y + np.sin(theta)*25, theta)
        xqs, yqs = np.zeros((1,180)), np.zeros((1,180))
        z_casts = np.zeros(180)
        if self.lookup is None:
            #print('None')
            start_angle = theta - np.pi/2
            angles = [(start_angle + n * np.pi/180)%(2*np.pi) for n in range(180)]
            #begin = time.time()
            for idx in range(0, 180, self.ray_step):
                # calculate ray cast for the particle @ that angle
                # create probability distribution
                # calculate likelihood for that laser reading
                angle = angles[idx]
                xq, yq, z_cast = self.range_find_one_angle(laser_pose, angle)
                xqs[0,idx] = xq
                yqs[0,idx] = yq
                z_casts[idx] = z_cast
                meas = z_t1_arr[idx]
                prob = self.getProbability(z_cast, meas)
                #print('z_cast = {}, meas = {}, prob = {}'.format(z_cast,meas,prob))
                q += np.log(prob)
            #print(time.time()-begin)
        else:
            #print('not None')
            x_map = int(x/10)
            y_map = int(y/10)
            start_angle = np.round((theta-np.pi/2)*180/np.pi).astype(int)
            angles = [(start_angle + n)%360 for n in range(180)]
            #begin = time.time()
            #for idx, angle in enumerate(angles):
            for idx in range(0, 180, self.ray_step):
                angle = angles[idx]
                z_cast = self.lookup[y_map,x_map,angle]
                if z_cast == -1:
                    print('wack')
                z_casts[idx] = z_cast
                meas = z_t1_arr[idx]
                prob = self.getProbability(z_cast, meas)
                q += np.log(prob)
                '''
                angle_rad = angle * np.pi/180
                dx = z_cast * np.cos(angle_rad)
                dy = z_cast * np.sin(angle_rad)
                xqs[0,idx] = np.round(x + dx).astype(int)
                yqs[0,idx] = np.round(y + dy).astype(int)
                '''
            #print(time.time()-begin)
        q = np.exp(q)
        return q, xqs, yqs, z_casts

    
    def range_find_one_angle(self, pose, angle):
        """ Find the ray cast at a pose at heading angle
            pose: in continuous coordinates
            angle: ray cast angle in radians
        """
        # robot heading not used since query angle is given
        x, y, _ = pose 
        sample_rate = 10
        z_cast = 0
        xq = np.round(x/10).astype(int)
        yq = np.round(y/10).astype(int)
        #while 0 <= self.map[yq,xq] < 0.5 and z_cast < self.range:
        while self.map[yq,xq] == 0 and z_cast < self.range:
            dx = z_cast * np.cos(angle)
            dy = z_cast * np.sin(angle)
            xq = np.round((x + dx)/10).astype(int)
            yq = np.round((y + dy)/10).astype(int)
            z_cast += sample_rate
        return xq, yq, z_cast

    def range_find_angles(self, pose, angles):
        """ Find the ray cast at a pose over multiple angles
            laser_pose:  in continuous coordinates
            angles: ray cast angles in radians
        """
        num_casts = len(angles)
        xqs, yqs = np.zeros(num_casts), np.zeros(num_casts)
        z_casts = np.zeros(num_casts)
        for i, angle in enumerate(angles):
            xq, yq, z_cast = self.range_find_one_angle(pose, angle)
            xqs[i] = xq
            yqs[i] = yq
            z_casts[i] = z_cast
        return xqs, yqs, z_casts

if __name__=='__main__':
    model = SensorModel(None)
    model.visualize()
