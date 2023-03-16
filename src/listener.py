#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

def callback(data):
    rospy.loginfo(data)
    
def gmapping_callback(data):
  dist = 8
  
  rospy.loginfo(data.info)
  
  delta = data.info.resolution
  width = data.info.width
  height = data.info.height
  pos_x = data.info.origin.position.x
  pos_y = data.info.origin.position.y
  
    
  grid = np.array(data.data)
  grid = grid.reshape((height, width))
  
  
  # plt.plot(-pos_x/delta, -pos_y/delta, marker="o")
  # plt.plot(-pos_x+6.5, -pos_y-1.8, marker="o")
  # plt.imshow(grid)
  # plt.show()

  # TODO rotate map with vehicle state
  
  x_min = int((-pos_x+6.5 - dist) / delta)
  x_max = int((-pos_x+6.5 + dist) / delta)
  y_min = int((-pos_y-1.8 - dist) / delta)
  y_max = int((-pos_y-1.8 + dist) / delta)
    
  local_grid = grid[y_min:y_max, x_min:x_max]
  
  # rospy.loginfo(local_grid)
  
  plt.imshow(local_grid)
  plt.show()

    
def listener():
    rospy.init_node('listener', anonymous=True)

    # rospy.Subscriber("/front/scan", LaserScan, callback)
    rospy.Subscriber("/map", OccupancyGrid, gmapping_callback)
    # rospy.Subscriber("/odom", Odometry, pos_callback)
    
    rospy.spin()

if __name__ == '__main__':
    listener()