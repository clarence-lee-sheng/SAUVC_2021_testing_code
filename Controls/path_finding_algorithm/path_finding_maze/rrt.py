import numpy as np 
import random

from numpy.random import rand 
from math import *
import pygame as pg

class RRT: 
    def __init__(self, maze, max_dist_threshold=800, target_boundary=70):
        self.maze = maze 
        self.parents = {tuple(maze.start_pos): None}
        self.points = np.array([np.array(maze.start_pos)])
        self.max_dist_threshold = max_dist_threshold
        self.target_boundary = target_boundary

    def compute_path(self): 
        target_node_found = False 
        while not target_node_found: 
            random_point = self.random_point()
            closest_point = self.get_closest_point(random_point)
            # if a point is not valid continue generating random points until a valid point is generated 
            if len(closest_point) == 0: 
                continue 
            self.parents[tuple(random_point)] = tuple(closest_point) 
            pg.draw.line(self.maze.path_surface,(50,50,200),closest_point, random_point,3) 
            pg.draw.circle(self.maze.path_surface,(255,128,0),random_point, 5) 
            self.maze.update()
            self.points = np.append(self.points,[random_point], axis=0)
            if self.euclid_dist(random_point, self.maze.target_pos) < self.target_boundary: 
                target_node_found = True 
                self.parents[tuple(self.maze.target_pos)] = tuple(random_point)
                
        current = tuple(self.maze.target_pos)
        parent = self.parents[current]
        path = [current]
        print("found")
        while parent:
            pg.draw.line(self.maze.path_surface,(255,255,255),current, parent,10) 
            current = parent 
            parent = self.parents[parent] 
            path.append(current)
            
        return path 

    # utility functions 
    def random_point(self):
        width = self.maze.width 
        height = self.maze.height 
        return np.array([random.random() * width, random.random()*height])
    
    def get_closest_point(self, point):
        print(self.points.shape)
        diff = point - self.points 
        
        # calculate euclidean distance point between selected point and all other points 
        euclid_distances = np.sqrt(np.sum(diff**2,1))
        closest_point_idx = euclid_distances.argmin() 
        closest_point =  self.points[closest_point_idx]
        # return False if distance between closest point and target point exceeds distance threshold or if the closest point is the point itself 
        if euclid_distances[closest_point_idx] >= self.max_dist_threshold or tuple(point) == tuple(closest_point): 
            return [] 
        else: 
            if self.maze.have_collision(closest_point,point):
                return []
            print(closest_point)
            return closest_point
    
    def euclid_dist(self, start, end): 
        start_x, start_y = start
        end_x, end_y = end 
        return sqrt((end_x-start_x)**2 + (end_y - start_y)**2)    


if __name__ == "__main__": 
    coordinates = np.array([[0,0],[1,2],[3,3],[4,6],[1,7],[8,3]])
    print(coordinates.shape)
    point = np.array([5,5])
    difference = point - coordinates
    print(np.sqrt(np.sum(difference ** 2,1)).min())


        

        


 
