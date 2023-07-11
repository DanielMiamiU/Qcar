# Copied from https://github.com/toolbuddy/2D-Grid-SLAM/blob/master/ParticleFilter.py
import numpy as np
from GridMap import *
import random
import math
import utils
import copy
import threading

class Particle:
    def __init__(self, pos, sensorSize, maxDist, mapUnits, gmap):
        self.pos = pos
        self.sensorSize = sensorSize
        self.maxDist = maxDist
        self.mapUnits = mapUnits
        self.gmap = gmap

    def Sampling(self, turnAngle, encoder_dist, sig=[0.2,0.2,0.1]):
        self.pos[0], self.pos[1] = self.pos[0] + utils.xPos(self.pos[2], encoder_dist) * self.mapUnits, self.pos[1] + utils.yPos(self.pos[2], encoder_dist) * self.mapUnits
        self.pos[2] = utils.radsLimit(self.pos[2] + utils.anglePos(self.pos[2], turnAngle, encoder_dist))

        self.pos[0] += random.gauss(0,sig[0])
        self.pos[1] += random.gauss(0,sig[1])
        self.pos[2] += random.gauss(0,sig[2])

    def NearestDistance(self, x, y, wsize, th):
        min_dist = 9999
        min_x = None
        min_y = None
        gsize = self.gmap.gsize
        xx = int(round(x/gsize))
        yy = int(round(y/gsize))
        for i in range(xx-wsize, xx+wsize):
            for j in range(yy-wsize, yy+wsize):
                if self.gmap.GetGridProb((i,j)) < th:
                    dist = (i-xx)*(i-xx) + (j-yy)*(j-yy)
                    if dist < min_dist:
                        min_dist = dist
                        min_x = i
                        min_y = j

        return math.sqrt(float(min_dist)*gsize)

    def LikelihoodField(self, angles, dists):
        p_hit = 0.9
        p_rand = 0.1
        sig_hit = 3.0
        q = 1
        plist = utils.EndPoint(self.pos, angles, dists)
        for i in range(len(plist)):
            if dists[i] >= self.maxDist * self.mapUnits or dists[i] < .01:
                continue
            dist = self.NearestDistance(plist[i][0], plist[i][1], 4, 0.2)
            #q = q * (p_hit*utils.gaussian(0,dist,sig_hit) + p_rand/(self.maxDist * self.mapUnits))
            
            q += math.log(p_hit*utils.gaussian(0,dist,sig_hit) + p_rand/(self.maxDist * self.mapUnits))
        return q

    def Mapping(self, num_measurements, angles, dists):
        for i in range(num_measurements):
            if dists[i] >= (self.maxDist) * self.mapUnits:
                continue
                """
                theta = self.pos[2] + angles[i]
                self.gmap.EmptyMapLine(
                int(self.pos[0]), 
                int(self.pos[0]+dists[i]*np.cos(theta)),
                int(self.pos[1]),
                int(self.pos[1]+dists[i]*np.sin(theta))
                )
                """
                
            if dists[i] < .05:
                continue
            theta = self.pos[2] - angles[i]
            self.gmap.GridMapLine(
            int(self.pos[0]), 
            int(self.pos[0]+dists[i]*np.cos(theta)),
            int(self.pos[1]),
            int(self.pos[1]+dists[i]*np.sin(theta))
            )

class ParticleFilter:
    #stuff to calc change in pos
    def __init__(self, pos, sensorSize, maxDist, mapUnits, gmap, size):
        self.size = size
        self.particle_list = []
        self.weights = np.ones((size), dtype=float) / size
        p = Particle(pos.copy(), sensorSize, maxDist, mapUnits, copy.deepcopy(gmap))
        for i in range(size):
            self.particle_list.append(copy.deepcopy(p))
    
    def ParticleMapping(plist, num_measurements, angles, dists):
        threads = []
        for p in plist:
            threads.append(threading.Thread(target=p.Mapping, args=(num_measurements, angles, dists,)))

        for t in threads:
            t.start()

        for t in threads:
            t.join()

    def Resampling(self, num_measurements, angles, dists):
        map_rec = np.zeros((self.size))
        re_id = np.random.choice(self.size, self.size, p=list(self.weights))
        new_particle_list = []
        for i in range(self.size):
            if map_rec[re_id[i]] == 0:
                self.particle_list[re_id[i]].Mapping(num_measurements, angles, dists)
                map_rec[re_id[i]] = 1
            new_particle_list.append(copy.deepcopy(self.particle_list[re_id[i]]))
        self.particle_list = new_particle_list
        self.weights = np.ones((self.size), dtype=float) / float(self.size)

    # sensor_data wanted = dsitances
    def Feed(self, turnAngle, encoder_dist, angles, dists):
        field = np.ones((self.size), dtype=float)
        for i in range(self.size):
            self.particle_list[i].Sampling(turnAngle, encoder_dist)
            field[i] = self.particle_list[i].LikelihoodField(angles, dists)
            #self.particle_list[i].Mapping(sensor_data)
        if (np.sum(field)!= 0):
            self.weights = field / np.sum(field)
        else:
            self.weight = np.ones((self.size), dtype=float) / self.size
        #self.Resampling(sensor_data)