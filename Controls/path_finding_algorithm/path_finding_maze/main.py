import pygame as pg
from math import sqrt 
import numpy as np
from rrt import RRT

class Maze: 
    def __init__(
        self, 
        path_algo,
        width = 800, 
        height = 600, 
        start_pos = (30,30), 
        target_pos = False, 
        radius = 20, 
        start_color = (0, 255, 0), 
        target_color = (255, 0, 0), 
        obstacles_color = (77, 135, 181),
        obstacles_radius = 10, 
    ):
        self.width = width 
        self.height = height
        self.screen = pg.display.set_mode((width, height))
        self.radius = radius
        self.start_pos = start_pos
        if target_pos: 
            self.target_pos = self.target_pos 
        else: 
            self.target_pos = (width - 30, height - 30)
        self.start_color = start_color 
        self.target_color = target_color
        self.obstacles_color = obstacles_color
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
        pg.draw.circle(screen, self.start_color, self.start_pos, self.radius)
        pg.draw.circle(screen, self.target_color, self.target_pos, self.radius)
        pg.display.flip()

    def event_handler(self, event): 
        # returns true if the mouse is over the goal 
        mouse_over_start_node = self.euclid_dist(self.start_pos, self.mouse_pos) < self.radius
        mouse_over_target_node = self.euclid_dist(self.target_pos, self.mouse_pos) < self.radius

        if event.type == pg.QUIT: 
            return "quit"
        elif event.type == pg.MOUSEBUTTONDOWN:
            if self.state == "path_computed": 
                return self.state
            if mouse_over_start_node: 
                return "start_node_clicked"
            elif mouse_over_target_node: 
                return "target_node_clicked"
            else: 
                return "draw_obstacles"

        elif event.type == pg.MOUSEBUTTONUP: 
            return "wait"
        elif event.type == pg.KEYDOWN: 
            if self.state == "path_computed": 
                self.path_surface.fill((0,0,0))
                return "wait"

            return "find_path"
        return self.state

    def run(self): 
        while True: 
            event = pg.event.poll() 
            mouse_pos = pg.mouse.get_pos()
            self.mouse_pos = mouse_pos

            state = self.event_handler(event)
            self.state = state
             
            if state == "quit": 
                return
            # mouse over start_node when click is registered 
            elif state == "start_node_clicked": 
                self.start_pos = mouse_pos
            # mouse over target_node when click is registered 
            elif state == "target_node_clicked": 
                self.target_pos = mouse_pos 
            # mouse over empty space when click is registered 
            elif state == "draw_obstacles":
                pg.draw.circle(self.obstacles_surface, self.obstacles_color, mouse_pos, self.obstacles_radius)
            elif state == "find_path": 
                self.path_surface.fill((0,0,0))
                path_algo = self.path_algo(self)
                path_algo.compute_path()
                self.state = "path_computed"

            maze.update()

    #############################################################################
    ########################### Utility methods #################################
    #############################################################################
    def euclid_dist(self, start, end): 
        start_x, start_y = start
        end_x, end_y = end 
        return sqrt((end_x-start_x)**2 + (end_y - start_y)**2)    

    def have_collision(self, src, dest): 
        diff = dest - src 
        unit_vector = diff/np.linalg.norm(diff)
        node = src 
        while self.euclid_dist(node, dest) > 1:
            # if the path is the colour of the obstacle, then a collision is detected
            if self.obstacles_surface.get_at((int(node[0]), int(node[1]))) == self.obstacles_color: 
                print("collision detected")
                return True 
            # slowly increment in the direction of the unit vector 
            node = np.array([node[0] + unit_vector[0], node[1] + unit_vector[1]])  
        return False 



        

if __name__ == "__main__": 
    maze = Maze(RRT) 
    maze.run()