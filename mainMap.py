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
# from SingleBotLaser2D import *
import copy
 
# map units is used to convert the lidar distances to different measurements (100 makes it cm)
num_measurements=720
max_distance=5
mapUnits=10
robot_pos = np.array([0.0, 0.0, 0.0])
myLidar = LIDAR(num_measurements, max_distance)
myCar = QCar()
gpad = gamepadViaTarget(1)
mySpeed = speedCalc(robot_pos, myCar, time.time())


startTime = time.time()
sampleTime = 1/10
def elapsed_time():
    return time.time() - startTime

#
def SensorMapping(m, bot_pos, angles, dists):
    for i in range(num_measurements):
        if dists[i] >= max_distance * mapUnits or dists[i] < .01:
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

def AdaptiveGetMap(gmap):
    
    mimg = gmap.GetMapProb(
        gmap.boundary[0]-20, gmap.boundary[1]+20, 
        gmap.boundary[2]-20, gmap.boundary[3]+20 )
    mimg = (255*mimg).astype(np.uint8)
    mimg = cv2.cvtColor(mimg, cv2.COLOR_GRAY2RGB)
    return mimg

new = gpad.read()

if __name__ == '__main__':
    #cv2.namedWindow('map', cv2.WINDOW_AUTOSIZE)
    #cv2.namedWindow('particle_map', cv2.WINDOW_AUTOSIZE)
    # Initialize GridMap
    # lo_occ, lo_free, lo_max, lo_min
    map_param = [.4, -.4, 5.0, -5.0] 
    m = GridMap(map_param, gsize=1)
    myLidar.read()

    

    # TODO change env to Qcar friendly
    while (elapsed_time() < 10.0):
        if myLidar.distances.any() != 0:
            SensorMapping(m, robot_pos, myLidar.angles, myLidar.distances * mapUnits)
            mimg = AdaptiveGetMap(m)
            
        
        myLidar.read()
    
    pf = ParticleFilter(robot_pos.copy(), num_measurements, max_distance, mapUnits, copy.deepcopy(m), 10)


    while gpad.B != 1:
        myLidar.read()
        new = gpad.read()
        start = time.time()
        mtr_cmd = np.array([.05*gpad.RT, (gpad.left - gpad.right) * .3])
        LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])
        

        myCar.read_write_std(mtr_cmd, LEDs)
        encoder_Dist = mySpeed.encoder_dist()
        robot_pos = utils.posUpdate(robot_pos, mtr_cmd[1], encoder_Dist)
        print(encoder_Dist)
        
        if (myLidar.distances.any() != 0):    
            #SensorMapping(m, robot_pos, myLidar.angles, myLidar.distances * mapUnits)
            #mimg = AdaptiveGetMap(m)
            #cv2.imshow('map',mimg)
            #cv2.waitKey(1)

            if (encoder_Dist > 0):
                pf.Feed(robot_pos[2], mtr_cmd[1], encoder_Dist * mapUnits, myLidar.angles, myLidar.distances * mapUnits)
                pf.Resampling(num_measurements, myLidar.angles, myLidar.distances * mapUnits)
                
                
                
        mid = np.argmax(pf.weights)
        imgp0 = AdaptiveGetMap(pf.particle_list[mid].gmap)
        cv2.imshow('particle_map',imgp0)
        cv2.waitKey(1)
        

        #robot_pos = pf.particle_list[mid].pos
        end = time.time()
        print(robot_pos)
        # Calculate the computation time, and the time that the thread should pause/sleep for
        computationTime = end - start
        sleepTime = sampleTime - ( computationTime % sampleTime )
        
        # Pause/sleep and print out the current timestamp
        time.sleep(sleepTime)


myLidar.terminate()
myCar.read_write_std(np.array([0,0]), np.array([0, 0, 0, 0, 0, 0, 0, 0]))