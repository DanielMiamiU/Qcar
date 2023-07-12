# Qcar SLAM
python code for use with the Quanser Q car.

## SLAM Overview
SLAM stands for Simultaneous Localization and Mapping. SLAM algorithms are notorious for being a "chicken and egg" problem, knowing your position in a map that is empty is very difficult.  

## Implementation
We are solving the problem with a grid based fastSLAM algorithm. The map that is being created by this code is a grid of probabilities representing the probability of an object in that grid cell. A particle filter is used to represent the motion of the car. A particle filter uses several praticles, which are a representation of the cars position. These particles also have a probability assigned to them, representing the probability the particle represents the actual position of the car. As the car moves, encoder data is transcribed to distance and sent to the particle filter. The particle filter moves the particles and uses a random number to simulate error. LIDAR data is used to build a map of the surroundings and also determine which particles is closest to the real position.


Code is appropriated from a simulation for use in the real world on the Qcar, github for the simluation is below.
https://github.com/toolbuddy/2D-Grid-SLAM/tree/master
