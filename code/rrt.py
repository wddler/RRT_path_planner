#! /usr/bin/env python
import csv
import os
import random
import numpy as np
import math

#define paths to files
cur_path = os.path.dirname('RRT.ipynb')
path_obstacles = os.path.relpath('..\\results\\obstacles.csv', cur_path)
path_nodes = os.path.relpath('..\\results\\nodes.csv', cur_path)
path_edges = os.path.relpath('..\\results\\edges.csv', cur_path)
path_path = os.path.relpath('..\\results\\path.csv', cur_path)

#function for write 3 files: nodes.csv, edges.csv, path.csv
def write_files(path):
    with open(path_path, mode='w') as path_file:
        path_writer = csv.writer(path_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        path_writer.writerow(path)
    with open(path_nodes, mode='w') as path_file:
        path_writer = csv.writer(path_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        path_writer.writerow(['#ID', 'X', 'Y'])
        for n in search_tree_T:
            path_writer.writerow([n.id, n.x, n.y])
    with open(path_edges, mode='w') as path_file:
        path_writer = csv.writer(path_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        path_writer.writerow(['#ID1', 'ID2', 'COST'])
        for e in edges_list:
            path_writer.writerow([e.id1, e.id2, e.cost])


class Obstacle:
    def __init__(self, x, y, diameter):
        self.x = x
        self.y = y
        self.diameter = diameter
        
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.id = None
        
class Edge:
    def __init__(self, ID1, ID2, cost):
        self.id1 = ID1
        self.id2 = ID2
        self.cost = cost

#function to make a sample X_samp using Python random function 
def sampling():
    x_coord = round(random.uniform(-0.5, 0.5), 5)
    y_coord = round(random.uniform(-0.5, 0.5), 5)
    return x_coord, y_coord

#function for finding nearest node to the x_samp
def find_nearest_to_x_samp(x_samp, search_tree_T):
    distances = {}
    for node in search_tree_T: 
        distance = np.sqrt(np.power((x_samp.x - node.x), 2) + np.power((x_samp.y - node.y), 2))
        distances[node] = distance
    min_distance = min(distances.values())
    closest = [k for k, v in distances.items() if v==min_distance]
    return closest[0]

#function to make x_new: from a nearest node towards x_samp 
def make_x_new(x_nearest, x_samp):
    dist = np.sqrt(np.power((x_samp.x - x_nearest.x), 2) + np.power((x_samp.y - x_nearest.y), 2)) #find Euclidian distance
    if dist < d: #if distance between x_nearest and x_samp is less than my step size "d", x_new = x_samp
        x_new = x_samp
    else:
        x_dir = x_samp.x - x_nearest.x
        y_dir = x_samp.y - x_nearest.y
        x = d * x_dir/dist + x_nearest.x
        y = d * y_dir/dist + x_nearest.y
        x_new = Node(x,y)
    return x_new

#function to check of a line intersects a circle
def circle_line_segment_intersection(circle_center, circle_radius, pt1, pt2, full_line=True, tangent_tol=1e-9):
    """ Find the points at which a circle intersects a line-segment.  This can happen at 0, 1, or 2 points.

    :param circle_center: The (x, y) location of the circle center
    :param circle_radius: The radius of the circle
    :param pt1: The (x, y) location of the first point of the segment
    :param pt2: The (x, y) location of the second point of the segment
    :param full_line: True to find intersections along full line - not just in the segment.  False will just return intersections within the segment.
    :param tangent_tol: Numerical tolerance at which we decide the intersections are close enough to consider it a tangent
    :return Sequence[Tuple[float, float]]: A list of length 0, 1, or 2, where each element is a point at which the circle intercepts a line segment.
    """

    (p1x, p1y), (p2x, p2y), (cx, cy) = pt1, pt2, circle_center
    (x1, y1), (x2, y2) = (p1x - cx, p1y - cy), (p2x - cx, p2y - cy)
    dx, dy = (x2 - x1), (y2 - y1)
    dr = (dx ** 2 + dy ** 2)**.5
    big_d = x1 * y2 - x2 * y1
    discriminant = circle_radius ** 2 * dr ** 2 - big_d ** 2

    if discriminant < 0:  # No intersection between circle and line
        return []
    else:  # There may be 0, 1, or 2 intersections with the segment
        intersections = [
            (cx + (big_d * dy + sign * (-1 if dy < 0 else 1) * dx * discriminant**.5) / dr ** 2,
             cy + (-big_d * dx + sign * abs(dy) * discriminant**.5) / dr ** 2)
            for sign in ((1, -1) if dy < 0 else (-1, 1))]  # This makes sure the order along the segment is correct
        if not full_line:  # If only considering the segment, filter out intersections that do not fall within the segment
            fraction_along_segment = [(xi - p1x) / dx if abs(dx) > abs(dy) else (yi - p1y) / dy for xi, yi in intersections]
            intersections = [pt for pt, frac in zip(intersections, fraction_along_segment) if 0 <= frac <= 1]
        if len(intersections) == 2 and abs(discriminant) <= tangent_tol:  # If line is tangent to circle, return just one point (as both intersections have same location)
            return [intersections[0]]
        else:
            return intersections

#function to check if a line intersects any obstacle from the obstacles list
def collision_check(x_nearest, x_new):
    for obs in obstacles_list:
        intersections = circle_line_segment_intersection((obs.x,obs.y),(obs.diameter/2 + 0.01),(x_nearest.x,x_nearest.y), (x_new.x, x_new.y))
        if intersections == []:
            collision_free_motion = True
        else:
            collision_free_motion = False
            break
    return collision_free_motion


#finction to check if we're already close to the X_goal
def almost_done(x_new):
    x = x_goal.x - x_new.x
    y = x_goal.y - x_new.y
    if x < d and y < d:
        return True
    else:
        return False

obstacles_list = []
edges_list = []
nodes_list = []

#read obstacles from the file obstacles.csv
with open(path_obstacles, newline='') as obstacles:
    obstacles_reader = csv.reader(obstacles)
    for row in obstacles_reader:
        if row[0][0] == '#': #skip the strings with comments
            continue
        else:
            obstacle_x = float(row[0])
            obstacle_y = float(row[1])
            obstacle_diameter = float(row[2])
            obstacles_list.append(Obstacle(obstacle_x, obstacle_y, obstacle_diameter))


#initialization of parameters
x_start = Node((-0.5), (-0.5))
x_start.id = 1
x_goal = Node((0.5), (0.5))
max_tree_size = 150
d = 0.1 #step
path = []
n_id = 1

print('Start finding a path')
print('max size of the search tree is', max_tree_size)

search_tree_T = [x_start] #initialize search tree T with x_start

while len(search_tree_T) < max_tree_size: #while T is less than the maximum tree size do
    x_samp = sampling() #sample from X
    x_samp = Node(x_samp[0],x_samp[1]) #crete object State 
    x_nearest = find_nearest_to_x_samp(x_samp, search_tree_T) # x_nearest = nearest node in T to x_samp
    # employ a local planner to find a motion from x_nearest to x_new in the direction of x_samp
    x_new = make_x_new(x_nearest, x_samp)
    no_collision = collision_check(x_nearest, x_new) #check collisions 
    if no_collision == True: # if the motion is collision-free then
        # add x_new to T with an edge from xnearest to xnew
        if x_new not in search_tree_T: #if the motion is collision-free then
            n_id = n_id + 1 #calculate id for the x_new
            x_new.id = n_id #assign id to the x_new
            x_new.parent = x_nearest #assign parent node
            search_tree_T.append(x_new) #add x_new to the search tree
            #create edge and put into the edges list:
            edges_list.append(Edge(x_new.id, x_nearest.id, np.sqrt(np.power((x_new.x - x_nearest.x), 2) + np.power((x_new.y - x_nearest.y), 2))))
        done = almost_done(x_new) #check if we're close to the x_goal
        if done == True: #if xnew is in Xgoal then
            success = True #return SUCCESS and the motion to xnew
            print('success =', success)
            node = x_new
            while node.parent != None: #reconsctruct path
                path.insert(0,(node.id))
                node = node.parent
			path.insert(0, 1)
            print(path)
            write_files(path) #write files
            break
            
if done != True:
    print('No path found. Try one more time')


    