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

from utils import get_occupancy_map
import time

def visualize_map(occupancy_map):
    fig = plt.figure()
    # plt.switch_backend('TkAgg')
    mng = plt.get_current_fig_manager();  # mng.resize(*mng.window.maxsize())
    #plt.ion(); 
    plt.imshow(occupancy_map, cmap='Greys'); 
    plt.axis([0, 800, 0, 800]);


def visualize_timestep(X_bar, tstep, xqs=None,yqs=None, xms=None, yms=None):
    x_locs = X_bar[:,0]/10.0
    y_locs = X_bar[:,1]/10.0
    scat = plt.scatter(x_locs, y_locs, c='r', marker='o', s=1)
    if xqs is not None:
        scat2 = plt.scatter(xqs/10, yqs/10, c='b', marker='o', s=1)
    if xms is not None:
        scat3 = plt.scatter(xms/10, yms/10, c='g', marker='o', s=1)

    plt.pause(50)
    #plt.pause(0.00002)
    plt.title('timestep {}'.format(tstep))
    plt.savefig('plots/time_{}'.format(tstep))
    scat.remove()
    if xqs is not None:
        scat2.remove()
    if xms is not None:
        scat3.remove()


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
    # ground truth start position
    '''
    y0_rand = np.random.uniform( 4000, 4010, (num_particles, 1) )
    x0_rand = np.random.uniform( 4150, 4160, (num_particles, 1) )
    #theta0_vals = np.random.uniform( np.pi-0.01, np.pi+0.01, (num_particles, 1) )
    '''


    theta0_vals = np.random.uniform( -np.pi,np.pi, (num_particles, 1) )
    theta0_vals[0][0] = np.pi
    new_num_particles = num_particles
    while len(y0_vals) < num_particles:
        # initialize [x, y] positions in world_frame for all particles
        y0_rand = np.random.uniform( 1000, 7010, (new_num_particles, 1) )
        x0_rand = np.random.uniform( 2000, 7160, (new_num_particles, 1) )

        for i in range(new_num_particles):
            y_map = np.round(y0_rand[i]/10.0).astype(np.int64)
            x_map = np.round(x0_rand[i]/10.0).astype(np.int64)
            if 0 <= occupancy_map[y_map, x_map] <= 0:
                if len(y0_vals) < num_particles:
                    y0_vals.append(y0_rand[i])
                    x0_vals.append(x0_rand[i])
        new_num_particles = num_particles-len(y0_vals)

    y0_vals = np.array(y0_vals)
    x0_vals = np.array(x0_vals)

    X_bar_init = np.hstack((x0_vals,y0_vals,theta0_vals,w0_vals))
    X_bar_init[0][0] = 4000
    X_bar_init[0][1] = 4400
    X_bar_init[0][2] = np.pi

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
    #lookup = np.load('lookup_zero.npy')

    motion_model = MotionModel()
    sensor_model = SensorModel(occupancy_map, lookup_flag=True)
    resampler = Resampling()

    num_particles = 5
    #X_bar = init_particles_random(num_particles, occupancy_map)
    X_bar = init_particles_freespace(num_particles, occupancy_map)
    #X_bar = np.array([[6500,1500,1*np.pi/2,1]])

    vis_flag = True
    count = 0

    """
    Monte Carlo Localization Algorithm : Main Loop
    """
    if vis_flag:
        visualize_map(occupancy_map)

    first_time_idx = True
    for time_idx, line in enumerate(logfile):
        #vis_flag = count % 1 == 0
        #vis_flag = False
        
        count += 1
            
        # Read a single 'line' from the log file (can be either odometry or laser measurement)
        meas_type = line[0] # L : laser scan measurement, O : odometry measurement
        meas_vals = np.fromstring(line[2:], dtype=np.float64, sep=' ') # convert measurement values from string to double

        odometry_robot = meas_vals[0:3] # odometry reading [x, y, theta] in odometry frame
        time_stamp = meas_vals[-1]

        #if ((time_stamp <= 0.0)): # ignore pure odometry measurements for now (faster debugging) 
        if ((time_stamp <= 0.0) or meas_type == "O"): # ignore pure odometry measurements for now (faster debugging) 
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
        num_rays = sensor_model.num_rays
        xqs = np.zeros((num_particles,num_rays))
        yqs = np.zeros((num_particles,num_rays))
        xms = np.zeros((num_particles,num_rays))
        yms = np.zeros((num_particles,num_rays))
        for m in range(0, num_particles):
            #print(m)
            """
            MOTION MODEL
            
            """
            x_t0 = X_bar[m, 0:3]
            #print('before motion: {}, {}, {}'.format(x_t0[0], x_t0[1], 180/np.pi*x_t0[2]))
            x_t1 = motion_model.update(u_t0, u_t1, x_t0)
            #print('after motion: {}, {}, {}'.format(x_t1[0], x_t1[1], 180/np.pi*x_t1[2]))
            x = int(x_t1[0]/10)
            y = int(x_t1[1]/10)
            
            if not(0 <= occupancy_map[y, x] <= 0.5) and meas_type == "L":
                #print('dumping particle')
                w_t = 0
                X_bar_new[m, :] = np.hstack((x_t1, w_t))
                continue

            """
            SENSOR MODEL
            """
            
            if (meas_type == "L"):
                z_t = ranges
                w_t, probs, z_casts, xqs[m], yqs[m], xms[m], yms[m] = sensor_model.beam_range_finder_model(z_t, x_t1)
                '''
                print('w_t = ',w_t)
                print(x, y, np.round(x_t1[2]*180/np.pi).astype(int))
                plt.figure()
                plt.scatter(np.arange(len(probs)),probs)
                plt.title('{}, {}, {} probability'.format(x, y, np.round(x_t1[2]*180/np.pi).astype(int)))
                plt.figure()
                plt.title('{}, {}, {} cast vs meas'.format(x, y, np.round(x_t1[2]*180/np.pi).astype(int)))
                plt.scatter(np.arange(len(z_casts)), z_casts, label='casts')
                z_t_plot= z_t[0:180:int(180/len(z_casts))]
                plt.scatter(np.arange(len(z_t_plot)), z_t_plot, label='measurements')
                plt.legend()
                '''
                X_bar_new[m,:] = np.hstack((x_t1, w_t))
            else:
                X_bar_new[m,:] = np.hstack((x_t1, X_bar[m,3]))
            #print(w_t)
        X_bar = X_bar_new
        u_t0 = u_t1

        if vis_flag:
            #xqs = np.reshape(xqs, (180, 1))
            #yqs = np.reshape(yqs, (180, 1))
            #X_bar = np.hstack((xqs,yqs))
            visualize_timestep(X_bar, time_idx, xqs, yqs, xms, yms)
            #visualize_timestep(X_bar, time_idx)

        """
        RESAMPLING
        """
        if (meas_type == "L"):
            print(X_bar[0])
            print(X_bar[30])
            X_bar = resampler.low_variance_sampler(X_bar)
        #X_bar = resampler.multinomial_sampler(X_bar)
         
def precompute_raycasts():
    # process map and get dimensions
    occupancy_map = get_occupancy_map()
    height, width = occupancy_map.shape
    print('height = {}, width = {}'.format(height, width))

    #initialize sensor model and begin ray casting
    sensor_model = SensorModel(occupancy_map)
    lookup = np.zeros((height,width,360))-1

    # x is col index and col = i % width
    # y is row index and row = i // width
    print('begin cast')
    theta_input = [theta*np.pi/180 for theta in range(360)]
    for i in tqdm(range(width*height)):
        x = i % width
        y = i // width
        x_map = x * 10
        y_map = y * 10
        #if not (0 <= occupancy_map[y,x] < 0.1):
        if occupancy_map[y,x] != 0:
            continue
        # range_find expects angle in radians
        xqs, yqs, z_casts = sensor_model.range_find_angles((x_map,y_map,0), theta_input)
        for theta, z_cast in zip(range(360), z_casts):
            lookup[y,x,theta] = z_cast
            #print('z_cast = {} at pose {}, {}, {}'.format(z_cast, x, y, theta))
    print('end cast')
    np.save('lookup_zero.npy', lookup)

def visualize_casts(pose):
    # get map and init sensor model
    occupancy_map = get_occupancy_map()
    sensor_model = SensorModel(occupancy_map)
    visualize_map(occupancy_map)

    x, y, theta = pose
    laser_pose = (x + np.cos(theta)*25, y + np.sin(theta)*25, theta)
    start_angle = theta - np.pi/2
    angles = [(start_angle + n * np.pi/180)%(2*np.pi) for n in range(180)]
    xqs, yqs, z_casts = sensor_model.range_find_angles(laser_pose, angles)
    print(z_casts)
    scat = plt.scatter(xqs, yqs, c='r', marker='o', s=1)
    plt.pause(100)


def visualize_casts_from_lookup(pose):
    ''' Visualize raycasts. 
        Input
            pose: expected in continuous frame (not map frame)
    '''

    lookup = np.load('lookup_zero.npy')
    print(lookup[0,0,0])
    occupancy_map = get_occupancy_map()
    height, width = occupancy_map.shape
    visualize_map(occupancy_map)

    # round theta to the nearest pi/180 increment
    x, y, theta = pose
    theta = np.round(theta * 180 / np.pi) * np.pi / 180
    x_laser = x + np.cos(theta) * 25
    y_laser = y + np.sin(theta) * 25
    start_angle = theta - np.pi / 2
    angles = [(start_angle + n * np.pi/180)%(2*np.pi) for n in range(180)]

    x_laser_map = int(x_laser/10)
    y_laser_map = int(y_laser/10)

    xqs = np.zeros((1,len(angles)))
    yqs = np.zeros((1,len(angles)))
    for i, angle in enumerate(angles):
        angle_deg = np.round(angle * 180 / np.pi).astype(int)
        z_cast = lookup[y_laser_map,x_laser_map,angle_deg]
        #print('{} at angle {}'.format(z_cast, angle_deg))
        xq = x_laser + z_cast * np.cos(angle)
        yq = y_laser + z_cast * np.sin(angle)
        xqs[0,i] = int(xq/10)
        yqs[0,i] = int(yq/10)

    scat = plt.scatter(xqs, yqs, c='r', marker='o', s=1)
    plt.pause(100)
    
  
if __name__=="__main__":
    main()
    #precompute_raycasts()
    #visualize_casts((4000,4000,1*np.pi/4))
    #visualize_casts_from_lookup((6500,1500,np.pi))

