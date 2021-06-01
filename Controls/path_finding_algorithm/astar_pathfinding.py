# Pseudo code for A* 
# Create two lists, an open list and a closed list 
# add the source node to the open list 
# for every adjacent node, calculate the hcost and gcost and therefore calculate the fcost 
# for each adjacent node, if it is blocked or out of grid or in the closed list, skip.
# else if the adjacent node is in the open list, check if the f_cost is lower, if it is 
# if f_cost is lower, replace the parent, and hcost and fcost 
# once reach target node, trace back all the parent nodes 

import heapq 
import math
from copy import deepcopy
from collections import defaultdict 

class Node: 
    def __init__(self,x,y,parent_node="",g_cost=0,h_cost=0): 
        self._x = x 
        self._y = y 
        self._parent_node = parent_node 
        self._g_cost = g_cost 
        self._h_cost = h_cost 
        self._f_cost = g_cost + h_cost 
    @property
    def x(self): 
        return self._x 
    @property 
    def y(self): 
        return self._y 
    @property 
    def parent_node(self): 
        return self._parent_node 
    @property 
    def g_cost(self):
        return self._g_cost
    @property 
    def h_cost(self):
        return self._h_cost
    @property
    def f_cost(self):
        return self._f_cost
    @x.setter
    def x(self,x): 
        self._x = x 
    @y.setter
    def y(self,y):
        self._y = y 
    @parent_node.setter
    def parent_node(self, parent_node): 
        self._parent_node = parent_node
    @g_cost.setter
    def g_cost(self, g_cost): 
        self._g_cost = g_cost 
        self._f_cost = self._g_cost + self._h_cost
    @h_cost.setter 
    def h_cost(self, h_cost): 
        self._h_cost = h_cost 
        self._f_cost = self._g_cost + self._h_cost
    @f_cost.setter
    def f_cost(self, f_cost): 
        self.f_cost = f_cost 
    def get_neighbour_coordinates(self): 
        masks = [(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1),(1,0),(-1,0)]
        neighbour_coords = map(lambda mask: (self.x + mask[0], self.y + mask[1]), masks)
        return neighbour_coords
    def calculate_distance(self, other_node): 
        return math.sqrt((other_node._x-self.x) ** 2 + (other_node._y-self.y) ** 2)
    def __lt__(self, other_node): 
        return self._f_cost < other_node.f_cost 



def astarSearch(grid, src_node, dest_node):
    searchGrid = deepcopy(grid)
    numRows = len(searchGrid)
    numCols = len(searchGrid[0])
    searched_nodes = defaultdict(lambda: defaultdict(int))
    open_list = []
    heapq.heappush(open_list, src_node)
    searched_nodes[src_node.x][src_node.y] = src_node
    dest_node_found = False
    while not dest_node_found: 
        current_node = heapq.heappop(open_list)
        searched_nodes[current_node.x][current_node.y] = [current_node]
        neighbourCoords = current_node.get_neighbour_coordinates()
        for coord in neighbourCoords: 
            x, y = coord
            new_node = Node(x, y)  
            new_node.h_cost = new_node.calculate_distance(dest_node)
            new_node.g_cost = current_node.calculate_distance(new_node) + current_node.g_cost  
            if x == dest_node.x and y == dest_node.y: 
                new_node.parent_node = current_node
                searched_nodes[x][y] = new_node
                dest_node_found = True
                break 
            if x < 0 or x >= numCols or y < 0 or y >= numRows: 
                continue 
            
            if searchGrid[y][x] == 0: 
                continue

            if searched_nodes[x][y]: 
                existing_node = searched_nodes[x][y][0]
                if existing_node.f_cost < new_node.f_cost: 
                    continue 
                new_node = existing_node
            
            new_node.parent_node = current_node
            heapq.heappush(open_list, new_node)        

    node = searched_nodes[dest_node.x][dest_node.y]
    while True: 
        prev_node = node.parent_node     
        print(prev_node)    
        if prev_node.x == src_node.x and prev_node.y == src_node.y: 
            searchGrid[src_node.y][src_node.x] = 3 
            searchGrid[dest_node.y][dest_node.x] = 3
            return searchGrid
        searchGrid[prev_node.y][prev_node.x] = 2 
        node = prev_node


if __name__ == "__main__": 
    grid = [[ 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 ],
            [ 1, 1, 1, 0, 1, 1, 1, 0, 1, 1 ],
            [ 1, 1, 1, 0, 1, 1, 0, 1, 0, 1 ],
            [ 0, 0, 1, 0, 1, 0, 0, 0, 0, 1 ],
            [ 1, 1, 1, 0, 1, 1, 1, 0, 1, 0 ],
            [ 1, 0, 1, 1, 1, 1, 0, 1, 0, 0 ],
            [ 1, 0, 0, 0, 0, 1, 0, 0, 0, 1 ],
            [ 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 ],
            [ 1, 1, 1, 0, 0, 0, 1, 0, 0, 1 ]]
    no_obstacles_grid = [[1 for j in range(10)] for i in range(10)]
    src_node = Node(0,8)
    dest_node = Node(9,0)
    final_grid = astarSearch(grid, src_node, dest_node)
    # final_grid = astarSearch(no_obstacles_grid, src_node, dest_node)
    for row in final_grid: 
        print(row)