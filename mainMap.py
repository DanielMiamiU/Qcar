from Quanser.q_essential import LIDAR
from Quanser.q_ui import gamepadViaTarget
from Quanser.product_QCar import QCar
from speedCalc import *
import numpy as np
import random
import utils
import cv2
import time
import math
from GridMap import *
from ParticleFilter import *
import copy


# num_measurements is how many positions the lidar scans for
# max_distance = maximum lidar scan in meters
# mapUnits is used to convert the lidar distances from meters to different measurements (10 makes it dm)
# robot_pos is the robots coordinates and pose [x, y, theta]
num_measurements=720
max_distance=2
mapUnits=40
robot_pos = np.array([0.0, 0.0, 0.0])

# Object initialization
myLidar = LIDAR(num_measurements, max_distance)
myCar = QCar()
gpad = gamepadViaTarget(1)
mySpeed = speedCalc(robot_pos, myCar, time.time())

# timing variables and helper function
startTime = time.time()
sampleTime = 1/50
def elapsed_time():
    return time.time() - startTime

# Changes the given map (m) based on robots position (bot_pos) and lidar scan results (angles and dists)
def SensorMapping(m, bot_pos, angles, dists):
    for i in range(num_measurements):
        if dists[i] >= (max_distance - .5) * mapUnits:
            theta = bot_pos[2] + angles[i]
            m.EmptyMapLine(
            int(bot_pos[0]), 
            int(bot_pos[0]+dists[i]*np.cos(theta)),
            int(bot_pos[1]),
            int(bot_pos[1]+dists[i]*np.sin(theta))
            )
            
            continue
        if dists[i] < .3:
            continue
        theta = bot_pos[2] + angles[i]
        m.GridMapLine(
        int(bot_pos[0]), 
        int(bot_pos[0]+dists[i]*np.cos(theta)),
        int(bot_pos[1]),
        int(bot_pos[1]+dists[i]*np.sin(theta))
        )
        # print(dists[i])
        # print(theta)

# Makes an image based on the given gridmap (gmap)
def AdaptiveGetMap(gmap):
    
    mimg = gmap.GetMapProb(
        gmap.boundary[0]-20, gmap.boundary[1]+20, 
        gmap.boundary[2]-20, gmap.boundary[3]+20 )
    mimg = (255*mimg).astype(np.uint8)
    mimg = cv2.cvtColor(mimg, cv2.COLOR_GRAY2RGB)

    return mimg

def DrawParticle(img, plist, scale=1.0):
    for p in plist:
        cv2.circle(img,(int(scale*p.pos[0]), int(scale*p.pos[1])), int(2), (0,200,0), -1)
    return img

new = gpad.read()

if __name__ == '__main__':
    #cv2.namedWindow('map', cv2.WINDOW_AUTOSIZE)
    
    cv2.namedWindow('particle_map', cv2.WINDOW_AUTOSIZE)
    # Initialize GridMap
    # lo_occ, lo_free, lo_max, lo_min
    map_param = [.4, -.4, 5.0, -5.0] 
    m = GridMap(map_param, gsize=1)
    myLidar.read()

    # Makes a rudimentary map of the starting area
    # Cannot move the car while making this map
    while (elapsed_time() < 5.0):
        if myLidar.distances.any() != 0:
            SensorMapping(m, robot_pos, myLidar.angles, myLidar.distances * mapUnits)
            mimg = AdaptiveGetMap(m)
        myLidar.read()
    
    # Initialize the particle filter based on the map of teh starting area
    pf = ParticleFilter(robot_pos.copy(), num_measurements, max_distance, mapUnits, copy.deepcopy(m), 10)

    # Main loop
    # Iterates until B is pressed on the gamepad
    while gpad.B != 1:
        myLidar.read()
        new = gpad.read()
        start = time.time()

        # RT controls speed of the car, Left stick controls truning left or right
        mtr_cmd = np.array([.07*gpad.RT, (gpad.left - gpad.right) * .3])
        LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])
        

        myCar.read_write_std(mtr_cmd, LEDs)
        encoder_Dist = mySpeed.encoder_dist()
        robot_pos = utils.posUpdate(robot_pos, mtr_cmd[1], mapUnits, encoder_Dist)
        #print(encoder_Dist)
        if (encoder_Dist > 0):
            print("Moving")
        
        # Lidar occasionally resets sensor values and returns 0 for measurements
        if (myLidar.distances.any() != 0):    
            #SensorMapping(m, robot_pos, myLidar.angles, myLidar.distances * mapUnits)
            #mimg = AdaptiveGetMap(m)
            #cv2.imshow('map',mimg)
            #cv2.waitKey(1)

            # Only update the particle filter when the car moves
            if (encoder_Dist > 0):
                pf.Feed(robot_pos[2], mtr_cmd[1], encoder_Dist, myLidar.angles, myLidar.distances * mapUnits)
                mid = np.argmax(pf.weights)
                print(pf.particle_list[mid].pos)
                pf.Resampling(num_measurements, myLidar.angles, myLidar.distances * mapUnits)
        else:
            print("Zeros")
                
                
        # Finds the most probable particle        
        mid = np.argmax(pf.weights)

        # Get an image from the most probable particle map
        imgp0 = AdaptiveGetMap(pf.particle_list[mid].gmap)
        imgp0 = DrawParticle(imgp0, pf.particle_list)
        cv2.imshow('particle_map',imgp0)
        cv2.waitKey(1)
        

        #robot_pos = pf.particle_list[mid].pos
        end = time.time()
        #print(robot_pos)
        # Calculate the computation time, and the time that the thread should pause/sleep for
        computationTime = end - start
        sleepTime = sampleTime - ( computationTime % sampleTime )
        
        # Pause/sleep and print out the current timestamp
        #time.sleep(sleepTime)


myLidar.terminate()
myCar.read_write_std(np.array([0,0]), np.array([0, 0, 0, 0, 0, 0, 0, 0]))