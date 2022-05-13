"""
ERC Project
Path-Planning
"""

import matplotlib.pyplot as plt
import random
import math
from shapely.geometry import Point, Polygon, LineString

def CheckIfValidPoint(point, node_to_connect, obstacles):
    is_valid = True
    for obj in obstacles:
        if(obj.polygon.contains(Point(point[0], point[1]))):
            is_valid = False
            break
    
        for j in range(len(obj.coord_list) - 1):
            line_a = LineString([point, (node_to_connect.x_coord, node_to_connect.y_coord)])
            line_b = LineString([(obj.coord_list[j][0], obj.coord_list[j][1]), (obj.coord_list[j+1][0], obj.coord_list[j+1][1])])
            if(line_a.intersects(line_b)):
                is_valid = False
                break

    return is_valid

def AddPointToTree(rand_point, min_index, nodes_in_tree, goal):
    found_goal = False

    nodes_in_tree.append(Nodes(rand_point[0], rand_point[1], min_index))

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
    
    nodes_in_tree = []
    nodes_in_tree.append(Nodes(start[0], start[1], 0))

    while (not found_goal):
        rand_point = SamplePoint(counter, bounds_of_plane, goal, sample_goal)
        counter = counter + 1
        
        min_dist = math.sqrt((rand_point[0] - start[0]) ** 2 + (rand_point[1] - start[1]) ** 2)
        min_index = 0
    
        for i in range(len(nodes_in_tree)):
            curr_dist = math.sqrt((rand_point[0] - nodes_in_tree[i].x_coord) ** 2 + (rand_point[1] - nodes_in_tree[i].y_coord) ** 2)
            if (curr_dist < min_dist):
                min_dist = curr_dist
                min_index = i
        
        if(min_dist > fixed_distance):
            rand_point[0] = (((rand_point[0] - nodes_in_tree[min_index].x_coord) * fixed_distance)/min_dist) + nodes_in_tree[min_index].x_coord
            rand_point[1] =  (((rand_point[1] - nodes_in_tree[min_index].y_coord) * fixed_distance)/min_dist) + nodes_in_tree[min_index].y_coord
        
        if(not CheckIfValidPoint(rand_point, nodes_in_tree[min_index], obstacles)):
            continue
        
        found_goal = AddPointToTree(rand_point, min_index, nodes_in_tree, goal)

        if(found_goal):
            AddPointToTree(goal_point, len(nodes_in_tree) - 1, nodes_in_tree, goal)

    return nodes_in_tree

def visualize(nodes_in_tree, obstacle_list):
  
    for obst in obstacle_list:
        obst.append(obst[0])
        xs, ys = zip(*obst)
        plt.plot(xs, ys)

    for node in nodes_in_tree:
        plt.plot([node.x_coord, nodes_in_tree[node.parent_index].x_coord], [node.y_coord, nodes_in_tree[node.parent_index].y_coord], "r.-", markersize = 3, linewidth = 0.3)

    curr_index = len(nodes_in_tree) - 1
    while(curr_index != 0):
        parent_index = nodes_in_tree[curr_index].parent_index
        plt.plot([nodes_in_tree[curr_index].x_coord, nodes_in_tree[parent_index].x_coord], [nodes_in_tree[curr_index].y_coord, nodes_in_tree[parent_index].y_coord], 'b.-', markersize = 5, linewidth = 0.5)
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
