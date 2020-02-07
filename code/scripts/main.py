import numpy as np
import sys
import pdb
from tqdm import tqdm

from MapReader import MapReader
from MotionModel import MotionModel
from SensorModel import SensorModel
from Resampling import Resampling

from matplotlib import pyplot as plt
from matplotlib import figure as fig
import time

def visualize_map(occupancy_map):
    fig = plt.figure()
    # plt.switch_backend('TkAgg')
    mng = plt.get_current_fig_manager();  # mng.resize(*mng.window.maxsize())
    plt.ion(); plt.imshow(occupancy_map, cmap='Greys'); plt.axis([0, 800, 0, 800]);


def visualize_timestep(X_bar, tstep):
    x_locs = X_bar[:,0]/10.0
    y_locs = X_bar[:,1]/10.0
    scat = plt.scatter(x_locs, y_locs, c='r', marker='o', s=1)
    #plt.pause(0.00001)
    plt.pause(100)
    scat.remove()

# TODO : change so that particles are not intialized in weird spots on map
def init_particles_random(num_particles, occupancy_map):

    # initialize [x, y, theta] positions in world_frame for all particles
    y0_vals = np.random.uniform( 0, 7000, (num_particles, 1) )
    x0_vals = np.random.uniform( 3000, 7000, (num_particles, 1) )
    theta0_vals = np.random.uniform( -3.14, 3.14, (num_particles, 1) )

    # initialize weights for all particles
    w0_vals = np.ones( (num_particles,1), dtype=np.float64)
    w0_vals = w0_vals / num_particles

    X_bar_init = np.hstack((x0_vals,y0_vals,theta0_vals,w0_vals))
    #print(w0_vals)    
    return X_bar_init

def init_particles_freespace(num_particles, occupancy_map):

    # initialize [x, y, theta] positions in world_frame for all particles

    """
    TODO : Add your code here
    """ 

    # initialize weights for all particles
    w0_vals = np.ones( (num_particles,1), dtype=np.float64)
    w0_vals = w0_vals / num_particles

    y0_vals = []
    x0_vals = []

    # initialize angle [theta] in world_frame for all particles
    theta0_vals = np.random.uniform( -3.14, 3.14, (num_particles, 1) )

    new_num_particles = num_particles
    while len(y0_vals) < num_particles:
        # initialize [x, y] positions in world_frame for all particles
        y0_rand = np.random.uniform( 0, 7000, (new_num_particles, 1) )
        x0_rand = np.random.uniform( 3000, 7000, (new_num_particles, 1) )

        for i in range(new_num_particles):
            if occupancy_map[np.round(y0_rand[i]/10.0).astype(np.int64), np.round(x0_rand[i]/10.0).astype(np.int64)] == 0:
                if len(y0_vals) < num_particles:
                    y0_vals.append(y0_rand[i])
                    x0_vals.append(x0_rand[i])

        new_num_particles = num_particles

    y0_vals = np.array(y0_vals)
    x0_vals = np.array(x0_vals)

    X_bar_init = np.hstack((x0_vals,y0_vals,theta0_vals,w0_vals))

    return X_bar_init

    
def main():

    """
    Description of variables used
    u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]   
    u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
    x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
    x_t1 : particle state belief [x, y, theta] at time t [world_frame]
    X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
    z_t : array of 180 range measurements for each laser scan
    """

    """
    Initialize Parameters
    """
    src_path_map = '../data/map/wean.dat'
    src_path_log = '../data/log/robotdata1.log'

    map_obj = MapReader(src_path_map)
    occupancy_map = map_obj.get_map() 
    logfile = open(src_path_log, 'r')

    motion_model = MotionModel()
    sensor_model = SensorModel(occupancy_map)
    resampler = Resampling()

    num_particles = 1
    #X_bar = init_particles_random(num_particles, occupancy_map)
    #X_bar = init_particles_freespace(num_particles, occupancy_map)
    X_bar = np.array([[6500,1500,0*np.pi/2,1]])

    vis_flag = True

    """
    Monte Carlo Localization Algorithm : Main Loop
    """
    if vis_flag:
        visualize_map(occupancy_map)

    first_time_idx = True
    for time_idx, line in enumerate(logfile):

        # Read a single 'line' from the log file (can be either odometry or laser measurement)
        meas_type = line[0] # L : laser scan measurement, O : odometry measurement
        meas_vals = np.fromstring(line[2:], dtype=np.float64, sep=' ') # convert measurement values from string to double

        odometry_robot = meas_vals[0:3] # odometry reading [x, y, theta] in odometry frame
        time_stamp = meas_vals[-1]

        if ((time_stamp <= 0.0) | (meas_type == "O")): # ignore pure odometry measurements for now (faster debugging) 
            continue

        if (meas_type == "L"):
             odometry_laser = meas_vals[3:6] # [x, y, theta] coordinates of laser in odometry frame
             ranges = meas_vals[6:-1] # 180 range measurement values from single laser scan
        
        print("Processing time step " + str(time_idx) + " at time " + str(time_stamp) + "s")

        if (first_time_idx):
            u_t0 = odometry_robot
            first_time_idx = False
            continue

        X_bar_new = np.zeros( (num_particles,4), dtype=np.float64)
        u_t1 = odometry_robot
        for m in range(0, num_particles):
            """
            MOTION MODEL
            """
            # print('start motion model')
            x_t0 = X_bar[m, 0:3]
            x_t1 = motion_model.update(u_t0, u_t1, x_t0)
            # print('end motion model')

            """
            SENSOR MODEL
            """
            
            if (meas_type == "L"):
                z_t = ranges
                w_t, xqs, yqs = sensor_model.beam_range_finder_model(z_t, x_t1)
                X_bar_new[m,:] = np.hstack((x_t1, w_t))
            else:
                X_bar_new[m,:] = np.hstack((x_t1, X_bar[m,3]))
        X_bar = X_bar_new
        u_t0 = u_t1


        """
        RESAMPLING
        """
        # X_bar = resampler.low_variance_sampler(X_bar)
        X_bar = resampler.multinomial_sampler(X_bar)
        if vis_flag:
            xqs = np.reshape(xqs, (len(xqs), 1))
            yqs = np.reshape(yqs, (len(yqs), 1))
            X_bar = np.hstack((xqs,yqs))
            visualize_timestep(X_bar, time_idx)
        print('end loop')
 
def precompute_raycasts():
    # process map and get dimensions
    src_path_map = '../data/map/wean.dat'
    map_obj = MapReader(src_path_map)
    occupancy_map = map_obj.get_map()
    height, width = occupancy_map.shape
    print('height = {}, width = {}'.format(height, width))

    #initialize sensor model and begin ray casting
    sensor_model = SensorModel(occupancy_map)
    lookup = np.zeros((height,width,360))-1

    # x is col index and col = i % width
    # y is row index and row = i // width
    print('begin cast')
    for i in tqdm(range(width*height*360)):
        y = i // width
        x = i % width
        if occupancy_map[y,x] != 0:
            continue
        for theta in range(360):
            # range_find expects angle in radians
            input_theta = theta / np.pi
            z_cast, xq, yq = sensor_model.range_find((x,y,input_theta), input_theta)
            lookup[y,x,theta] = z_cast
    print('end cast')
    np.save('lookup.npy', lookup)

  
if __name__=="__main__":
    #main()
    precompute_raycasts()

