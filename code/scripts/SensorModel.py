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
        pass

    # calculates the probability of your measurement given sensor model pdf based on the raycasted mean val
    def getProbability(self, mean, measurement):
        pass

    def beam_range_finder_model(self, z_t1_arr, x_t1):
        """
        param[in] z_t1_arr : laser range readings [array of 180 values] at time t
        param[in] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        param[out] prob_zt1 : likelihood of a range scan zt1 at time t
        """

        """
        TODO : Add your code here
        """
        q=1
        # picking every other measurement for now
        for i in range(len(z_t1_arr)/2):
            # calculate ray cast for the particle @ that angle
            # create probability distribution
            # calculate likelihood for that laser reading
            pass
            
        # multiply all the likelihoods

        return q    
 
if __name__=='__main__':
    pass