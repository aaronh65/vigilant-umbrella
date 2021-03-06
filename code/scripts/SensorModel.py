import numpy as np
import math
import time
from matplotlib import pyplot as plt
from scipy.stats import norm
import pdb
from utils import get_occupancy_map
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
        self.w1 = 1500
        self.w2 = 1750
        self.w3 = 1500
        self.w4 = 250
        self.sigma = 90
        self.norm = norm
        self.lmbda = 150
        self.map = occupancy_map
        self.range = 8183
        self.ray_step = 5
        if 180%self.ray_step != 0:
            print('ray step not even divisible with 180 ray casts')
        self.num_rays = int(180/self.ray_step)
        if lookup_flag:
            self.lookup = np.load('lookup_zero.npy')
        else:
            self.lookup = None

        self.rad2deg = lambda rad: np.round(rad*180/np.pi).astype(int)%360
        self.deg2rad = lambda deg: deg*np.pi/180%(2*np.pi)


    def visualize_dist(self):
        measurements = np.arange(0,self.range,1) + 1
        cast = np.arange(0, self.range,1)
        cast = 3000

        probs = [self.getProbability(cast, meas) for meas in measurements]
        plt.plot(measurements, probs)
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
            gauss_norm = self.norm.cdf(self.range, loc=ray, scale=self.sigma) - self.norm.cdf(0,loc=ray, scale=self.sigma)
            gauss = self.norm.pdf(meas,loc=ray, scale=self.sigma) / gauss_norm
        else:
            gauss = 0

        # short
        if 0 < meas < ray:
            exp = self.lmbda*np.exp(-self.lmbda*meas)
            exp *= 1/(1-np.exp(-self.lmbda*ray))
        else:
            exp = 0

        # out of range
        if meas >= self.range:
            p_max = 1
        else:
            p_max = 0

        # random
        if (meas > 0 and meas < self.range):
            p_rand = 1/self.range
        else:
            p_rand = 0
        p = self.w1*gauss + self.w2*exp + self.w3*p_max + self.w4*p_rand
        p /= (self.w1 + self.w2 + self.w3 + self.w4)
        #print(p)
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
        x_map, y_map = int(x/10), int(y/10)
        '''
        # non lookup
        laser_pose = (x + np.cos(theta)*25, y + np.sin(theta)*25, theta)
        xqs, yqs = np.zeros(self.num_rays), np.zeros(self.num_rays)
        xms, yms = np.zeros(self.num_rays), np.zeros(self.num_rays)
        z_casts = np.zeros(self.num_rays)
        '''
        probs = np.zeros(self.num_rays)
        start_angle = theta - np.pi/2
        angles = [(start_angle + self.deg2rad(n))%(2*np.pi) for n in range(0,180,self.ray_step)]
        angles_map = [self.rad2deg(angle) for angle in angles]
        z_casts = self.lookup[y_map,x_map,angles_map]
        xqs = x + np.cos(angles) * z_casts
        yqs = y + np.sin(angles) * z_casts
        z_scans = [z_t1_arr[n] for n in range(0,180,self.ray_step)]
        xms = x + np.cos(angles) * z_scans
        yms = y + np.sin(angles) * z_scans
        for idx in range(self.num_rays):
            probs[idx] = self.getProbability(z_casts[idx], z_scans[idx])
            q += np.log(probs[idx])
        q = 1.0/np.abs(q)**1.0
        #print(q)
        return q, probs, z_casts, xqs, yqs, xms, yms
        
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
    map = get_occupancy_map()
    model = SensorModel(map)
    model.visualize_dist()

