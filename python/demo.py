#! /usr/bin/env python3

# #{ imports

from matplotlib import colors
import rospy # the ROS api for python.
import roscpp # the ROS api for python.
import os #read system variable.
import numpy as np
from math import pow, atan2, sqrt
import time
import matplotlib.pyplot as plt
from occupancy_grid import occupancygrid_to_numpy
from d_star_lite import DStarLite
from grid import OccupancyGridMap, SLAM


i = 0


class Robot:

    # #{ __init__()

    def __init__(self):
        pass

        
 

    ## | ------------------------- methods ------------------------ |

    
    # # Converter occupancy grid to 2D array
    def occ_grid_to_2d(self):
        return occupancygrid_to_numpy(self.map)
    
    

    # #
    def ConvertMapIndicesToMeters(self,ia,ib):
        info = self.map.info
        resolution = info.resolution
        x = info.origin.position.x + ib*resolution
        y = info.origin.position.y + ia*resolution
        return x,y


    # #
    def ConvertMetersToMapIndices(self,x,y):
        info = self.map.info
        resolution = info.resolution
        # This code hasn't been checked and might not round to the closest index
        ib = int((x - info.origin.position.x)/resolution)
        ia = int((y - info.origin.position.y)/resolution) 
        return ia,ib


   
        
    # # D Star Lite 
    def Demo(self):
        
        OBSTACLE = 255
        UNOCCUPIED = 0

        x_dim = 328
        y_dim = 328
        start = (164,164)
        goal = (234,164)
        view_range = 10

        map_2D = np.zeros(shape=(x_dim,y_dim))
        map_2D[180:190, 160:170] = 100
        
        plt.scatter(start[0],start[1], marker='s', c='blue' )
        plt.scatter(goal[0], goal[1], marker='s', c='blue' )


        plt.imshow(map_2D)
        plt.pause(2)
        


        # create occupancy grid map
        new_map = OccupancyGridMap(x_dim=x_dim, y_dim=y_dim, exploration_setting='8N')
        old_map = new_map

        new_position = start
        last_position = start


        # D* Lite (optimized)
        dstar = DStarLite(map=new_map,s_start=start,s_goal=goal)
        

        # SLAM to detect vertices
        slam = SLAM(map=new_map,view_range=view_range)
       
        
        # move and compute path
        path, g, rhs = dstar.move_and_replan(robot_position=new_position)
       

        # Upgrade obstacle all map                    << problem printing rotated obstacles
        for row in range(y_dim):
            for col in range(x_dim):
                if map_2D[row][col]   == 100:
                    new_map.set_obstacle((row,col))
                    plt.scatter(row,col, marker='x', c='red' )
                elif map_2D[row][col] == 0:
                    new_map.remove_obstacle((row,col)) 
                else:
                    pass
        


        pos_path=0

        while True:   #< TODO check goal position
        # update the map
        # print(path)
        # drive gui
        

            new_position = (path[pos_path+1][0],path[pos_path+1][1])
            # new_observation = None
            new_map = OccupancyGridMap(x_dim=x_dim, y_dim=y_dim, exploration_setting='8N')

            """
            if new_observation is not None:
                if new_observation["type"] == OBSTACLE:
                    dstar.global_map.set_obstacle(pos=new_observation["pos"])
                if new_observation["pos"] == UNOCCUPIED:
                    dstar.global_map.remove_obstacle(pos=new_observation["pos"])
            """

            # if new_observation is not None:
            #     old_map = new_map
            #     slam.set_ground_truth_map(gt_map=new_map)
            print(new_position)

            if new_position != last_position:
                last_position = new_position
                if new_position == goal:
                    print("G O A L")
                    break
                plt.scatter(path[i][0],path[i][1], c="green")
                plt.draw
                plt.pause(0.001)
                plt.show

                # slam
                new_edges_and_old_costs, slam_map = slam.rescan(global_position=new_position)

                dstar.new_edges_and_old_costs = new_edges_and_old_costs
                dstar.sensed_map = slam_map

                # d star
                path, g, rhs = dstar.move_and_replan(robot_position=new_position)





        
   
    
if __name__ == "__main__":
   
    x = Robot()
    x.Demo()



