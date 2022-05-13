"""
ERC Project
Path-Planning
"""

import matplotlib.pyplot as plt
import random
import math
from shapely.geometry import Point, Polygon, LineString

def validPoint(point, connect_nodes, obstacles):
    is_valid = True
    for obj in obstacles:
        if(obj.polygon.contains(Point(point[0], point[1]))):
            is_valid = False
            break
    
        for j in range(len(obj.coord_list) - 1):
            line_a = LineString([point, (connect_nodes.x_coord, connect_nodes.y_coord)])
            line_b = LineString([(obj.coord_list[j][0], obj.coord_list[j][1]), (obj.coord_list[j+1][0], obj.coord_list[j+1][1])])
            if(line_a.intersects(line_b)):
                is_valid = False
                break

    return is_valid

def addPoint(rand_point, min_index, tree_nodes, goal):
    found_goal = False

    tree_nodes.append(Nodes(rand_point[0], rand_point[1], min_index))

    if(goal.polygon.contains(Point(rand_point[0], rand_point[1]))):
        found_goal = True

    return found_goal

def SamplePoint(counter, bounds_of_plane, goal, sample_goal):

    rand_point = [random.uniform(0, bounds_of_plane[0]), random.uniform(0, bounds_of_plane[1])]

    if(counter % sample_goal == 0):
        min_x, min_y, max_x, max_y = goal.polygon.bounds
        rand_point = [random.uniform(min_x, max_x), random.uniform(min_y, max_y)]
    
    return rand_point

class Figures:

    def __init__(self, coord_list):
        self.coord_list = coord_list
        self.polygon = Polygon(coord_list)

class Nodes:

    def __init__(self, x_coord, y_coord, parent_index):
        self.x_coord = x_coord
        self.y_coord = y_coord
        self.parent_index = parent_index

def RRT(start, goal_point, obstacle_list):
  
    bounds_of_plane = (100, 100)
    fixed_distance = 0.25 
    sample_goal = 5
    counter = 0 
    found_goal = False

    obstacles = []
    for obst in obstacle_list:
        obst.append(obst[0])
        obstacles.append(Figures(obst))

    goal = Figures([(goal_point[0], goal_point[1]), (goal_point[0], goal_point[1] - 0.2), (goal_point[0] - 0.2, goal_point[1] - 0.2), (goal_point[0] - 0.2, goal_point[1])])
    
    tree_nodes = []
    tree_nodes.append(Nodes(start[0], start[1], 0))

    while (not found_goal):
        rand_point = SamplePoint(counter, bounds_of_plane, goal, sample_goal)
        counter = counter + 1
        
        min_dist = math.sqrt((rand_point[0] - start[0]) ** 2 + (rand_point[1] - start[1]) ** 2)
        min_index = 0
    
        for i in range(len(tree_nodes)):
            curr_dist = math.sqrt((rand_point[0] - tree_nodes[i].x_coord) ** 2 + (rand_point[1] - tree_nodes[i].y_coord) ** 2)
            if (curr_dist < min_dist):
                min_dist = curr_dist
                min_index = i
        
        if(min_dist > fixed_distance):
            rand_point[0] = (((rand_point[0] - tree_nodes[min_index].x_coord) * fixed_distance)/min_dist) + tree_nodes[min_index].x_coord
            rand_point[1] =  (((rand_point[1] - tree_nodes[min_index].y_coord) * fixed_distance)/min_dist) + tree_nodes[min_index].y_coord
        
        if(not validPoint(rand_point, tree_nodes[min_index], obstacles)):
            continue
        
        found_goal = addPoint(rand_point, min_index, tree_nodes, goal)

        if(found_goal):
            addPoint(goal_point, len(tree_nodes) - 1, tree_nodes, goal)

    return tree_nodes

def visualize(tree_nodes, obstacle_list):
  
    for obst in obstacle_list:
        obst.append(obst[0])
        xs, ys = zip(*obst)
        plt.plot(xs, ys)

    for node in tree_nodes:
        plt.plot([node.x_coord, tree_nodes[node.parent_index].x_coord], [node.y_coord, tree_nodes[node.parent_index].y_coord], "r.-", markersize = 3, linewidth = 0.3)

    curr_index = len(tree_nodes) - 1
    while(curr_index != 0):
        parent_index = tree_nodes[curr_index].parent_index
        plt.plot([tree_nodes[curr_index].x_coord, tree_nodes[parent_index].x_coord], [tree_nodes[curr_index].y_coord, tree_nodes[parent_index].y_coord], 'b.-', markersize = 5, linewidth = 0.5)
        curr_index = parent_index

    plt.show()

def test_rrt():
    
    obstacle_list = [
      [(40, 0), (40, 40), (50, 50), (60, 40), (50, 40)],
      [(10, 10), (20, 20), (10, 30), (0, 20)],
      [(50, 60), (70, 80), (60, 100), (40, 80), (45, 100)],
      [(70, 20), (90, 20), (80, 40)]
    ]
    
    start = (1, 1)
    goal = (100, 1)
    
    path = RRT(start,goal, obstacle_list)
    visualize(path, obstacle_list)

test_rrt()
