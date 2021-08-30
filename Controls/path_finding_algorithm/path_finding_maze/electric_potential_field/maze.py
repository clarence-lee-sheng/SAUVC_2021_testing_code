import pygame as pg 
import math 
import numpy as np 
from electric_potential_fields import epf 

class Maze:
    def __init__(
        self,
        path_algo, 
        width = 800, 
        height = 600, 
        start_pos = (30,30)
        radius = 20, 
        start_color = (0,255, 0), 
        target_color = (255, 0, 0), 
        obstacles_color = (77, 135, 181), 
        obstacles_radius = 10    
    ): 

        self.width = width
        self.height = height
        self.screen  = pg.display.set_mode((width, height))
        self.radius = radius 
        self.start_pos = start_pos 
        self.start_color = start_color 
        self.target_color = target_color 
        self.obstacles_radius = obstacles_radius
        self.state = "wait"
        self.mouse_pos = (0,0)
        self.obstacles_surface = pg.Surface((width, height))
        self.path_surface = pg.Surface((width, height))
        self.path_surface.set_colorkey((0,0,0))
        self.path_algo = path_algo 

    def update(self): 
        screen = self.screen 
        screen.fill(0)
        screen.blit(self.obstacles_surface, (0,0))
        screen.blit(self.path_surface, (0,0))
        pg.draw_circle(screen, self.start_color, self.start_pos, self.radius)
        pg.draw_circle(screen, self.target_color, self.target_pos, self.radius)
        pg.display() 

    def event_handler(self, event): 
        mouse_over_start_node = self.euclid_dist(self.start_pos, self.mouse_pos) < self.radius 
        mouse_over_gate_pos = self.euclid_dist(self.gate_pos[0], self.mouse_pos) < self.radius or self.euclid_dist(self.gate_pos[1], self.mouse_pos) < self.radius

        if event.type == pg.MOUSEBUTTONDOWN: 
            if self.state == "target_reached": 
                return self.state 
            if mouse_over_start_node: 
                return "start_node"

    def euclid_dist(self, start, end): 
        start_x, start_y = start 
        end_x, end_y = end 
        return math.sqrt((end_x-start_x)**2 + (end_y-start_y)**2)

    def have_collision(self, src, dest): 
        diff = dest - src 
        unit_vector = diff/np.linalg.norm(diff)
        node = src 
        while self.euclid_dist(node, dest) > 1: 
