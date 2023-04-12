#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from skimage.draw import draw

class OccupancyGridNode:
    def __init__(self):
        rospy.init_node('occupancy_grid_node')

        self.resolution = 0.1 # grid resolution in meters
        self.width = 100 # grid width in cells
        self.height = 100 # grid height in cells
        self.origin_x = -5.0 # origin x in meters
        self.origin_y = -5.0 # origin y in meters
        self.grid = np.zeros((self.width, self.height), dtype=np.int8) # occupancy grid
        self.map_msg = OccupancyGrid() # occupancy grid message
        self.map_msg.header.frame_id = 'base_link' # frame id
        self.map_msg.info.resolution = self.resolution # resolution in meters/cell
        self.map_msg.info.width = self.width # width in cells
        self.map_msg.info.height = self.height # height in cells
        self.map_msg.info.origin.position.x = self.origin_x # origin x in meters
        self.map_msg.info.origin.position.y = self.origin_y # origin y in meters
        self.map_msg.info.origin.orientation.w = 1.0 # orientation as a quaternion

        rospy.Subscriber('/front/scan', LaserScan, self.scan_callback) # subscribe to LaserScan topic
        self.pub = rospy.Publisher('/local_map', OccupancyGrid, queue_size=1) # publish occupancy grid

    def scan_callback(self, scan_msg):
        
        angles = np.arange(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment) # laser scan angles
        ranges = np.array(scan_msg.ranges) # laser scan ranges        
        
        ranges = ranges[:-1] # fix dimensions
        ranges[ranges > 10] = 10 # max range = 10m
        
        x = np.sin(angles) * ranges # x position in the robot frame
        y = np.cos(angles) * ranges # y position in the robot frame
        
        x_map = ((x - self.origin_x) / self.resolution).astype(int) # x position in the map frame
        y_map = ((y - self.origin_y) / self.resolution).astype(int) # y position in the map frame
        
        
        valid_occupied = np.where((x_map >= 0) & (x_map < self.width) & (y_map >= 0) & (y_map < self.height))[0] # valid grid indices
        
        self.grid.fill(-1) # fill the grid with unknown values
        
        # fill in unoccupied cells with zeros
        for i in range(len(x_map)):
          rr, cc = draw.line(self.width//2, self.height//2, x_map[i], y_map[i])
          
          valid_unoccupied = np.where((rr >= 0) & (rr < self.width) & (cc >= 0) & (cc < self.height))    
          self.grid[rr[valid_unoccupied], cc[valid_unoccupied]] = 0
                
        self.grid[x_map[valid_occupied], y_map[valid_occupied]] = 100 # mark obstacles as occupied

        self.map_msg.data = self.grid.ravel().tolist() # convert grid to a 1D array and store it in the occupancy grid message
        self.pub.publish(self.map_msg) # publish the occupancy grid

if __name__ == '__main__':
    node = OccupancyGridNode()
    rospy.spin()
