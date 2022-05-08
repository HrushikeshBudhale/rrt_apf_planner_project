# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# Created By  : Pratik Acharya, Hrushikesh Budhale
# Created Date: Saturday 7 May 2022
# =============================================================================


# =============================================================================
# Imports
# =============================================================================

import matplotlib.pyplot as plt
from scipy import ndimage
import numpy as np
np.seterr(divide='ignore', invalid='ignore')
import math
import time
import cv2
import sys

# =============================================================================
# RRT + APF class
# =============================================================================

class RrtApf:
    def __init__(self, start, goal, obs_map, offset=3, maxIter=5000, goal_radius=15, animate=False):
        self.start = start
        self.goal = goal
        self.animate = animate                      # boolean variable to show expanding rrt search tree
        self.goal_probability = 0.1                 # probability to select goal node while selecting random nodes
        self.maxIter = maxIter                      # maximum number of iterations before returning 'path not found'
        self.nodes = [start]                        # stores visited nodes while exploring the map
        self.goal_radius = goal_radius              # vicinity aroung goal to consider path to be complete
        self.offset = offset                        # distance between selected random nodes and it's 2 neighbots
        self.came_from = {tuple(self.start): None}  # dictionary for back tracking found path
        self.distance_mat = ndimage.distance_transform_edt(obs_map/255 == 0)    # stores distance of each node from obstacle
        self.distance_mat = self.distance_mat / 40 + 0.00000001

    def get_nearest_node(self, node):
        min_dist = np.inf
        nearest_node = None
        for node_i in self.nodes:
            dist = np.linalg.norm(node - node_i)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node_i
        return nearest_node

    def calculate_apf(self, valid_nodes, attraction=0.5, repulsion=0.5):
        least_potential = np.inf
        new_node = None

        for node in valid_nodes:
            distance_to_goal = np.linalg.norm(node - self.goal)
            positive_potential = attraction * distance_to_goal
            negative_potential = repulsion / self.distance_mat[node[1]][node[0]]
            total_potential = positive_potential + negative_potential
            if total_potential < least_potential:
                least_potential = total_potential
                new_node = node

        return new_node

    def get_random_node(self):
        if np.random.random() < self.goal_probability:
            return np.array(self.goal)
        x = np.random.randint(0, self.distance_mat.shape[1])
        y = np.random.randint(0, self.distance_mat.shape[0])
        return np.array([x, y])

    def get_new_node(self, node, parent_node):

        if np.array_equal(node, parent_node): return node

        direction = node - parent_node
        direction_norm = (direction / np.linalg.norm(direction)) * self.offset
        new_node_center = (parent_node + direction_norm).round().astype(int)

        dx = dy = 0
        if (node[0] - parent_node[0]) == 0: dx = -self.offset
        elif (node[1] - parent_node[1]) == 0: dy = -self.offset
        else:
            slope = (node[1] - parent_node[1]) / (node[0] - parent_node[0])
            dy = math.sqrt(self.offset ** 2 / (slope ** 2 + 1))
            dx = -slope * dy

        new_node_left = new_node_center + np.array([dx, dy])
        new_node_right = new_node_center - np.array([dx, dy])
        selected_nodes = np.array([new_node_center, new_node_left, new_node_right]).round().astype(int)

        valid_nodes = selected_nodes[np.asarray([[not check_if_invalid(node_i[0], node_i[1])] for node_i in
                                                  selected_nodes]).flatten()]

        return self.calculate_apf(valid_nodes) if len(valid_nodes) != 0 else None

    def back_track(self, new_node):
        path = [self.goal]
        while new_node is not None:
            path.append(new_node)
            new_node = self.came_from[tuple(new_node)]
        path.reverse()
        return np.array(path).reshape(-1,2)

    def find_path(self):
        for iteration in range(self.maxIter):
            random_node = self.get_random_node()
            nearest_node = self.get_nearest_node(random_node)

            new_node = self.get_new_node(random_node, nearest_node)
            if (new_node is None) or np.array_equal(new_node, nearest_node):
                continue
            
            # when path reaches goal location
            if np.linalg.norm(new_node - self.goal) < self.goal_radius:
                self.came_from[tuple(new_node)] = nearest_node
                print(f"Path found in {iteration} iterations")
                return self.back_track(new_node), self.nodes
                
            match_found = np.any(np.all(new_node == self.nodes, axis=1))
            if not match_found:
                self.came_from[tuple(new_node)] = nearest_node
                self.nodes.append(new_node)
                if self.animate:
                    ax.plot(new_node[0], new_node[1], 'b.', markersize=2)
                    plt.pause(0.001)

        print("Path not found")
        sys.exit()

# =============================================================================
# Helper functions
# =============================================================================

def check_if_invalid(x, y):
    return x < 0 or x >= conf.shape[1] or y < 0 or y >= conf.shape[0] or conf[y, x] != 0

def check_in_poly(pts, poly):
    count = np.zeros(pts.shape[0])
    for i, _ in enumerate(poly[:-1]):
        # pts.y should be within y limits of line and pts.x should be less than intersection.x
        intersection_x = (poly[i+1,0] - poly[i,0]) * (pts[:,1]-poly[i,1]) / (poly[i+1,1] - poly[i,1]) + poly[i,0]
        count += (((pts[:,1] > poly[i,1]) != (pts[:,1] > poly[i+1,1])) & (pts[:,0] < intersection_x))*1
    return count % 2 # point is outside if even number of intersections

def create_rect(cx, cy, ht, wd):
    return np.array([(cx-(wd/2), cy-(ht/2)), (cx-(wd/2), cy+(ht/2)),
                     (cx+(wd/2), cy+(ht/2)), (cx+(wd/2), cy-(ht/2)),
                     (cx-(wd/2), cy-(ht/2))])

def get_obstacle_map(pts):
    # add square
    obs = check_in_poly(pts, RES*create_rect(1, 5, 1.5, 1.5).reshape(-1,2))

    # add rectangle 1
    obs2 = check_in_poly(pts, RES*create_rect(5, 5, 1.5, 2.5).reshape(-1,2))
    obs = np.logical_or(obs, obs2)
    
    # add rectangle 2
    obs2 = check_in_poly(pts, RES*create_rect(8, 3, 2, 1.5).reshape(-1,2))
    obs = np.logical_or(obs, obs2)

    # add circle 1
    center, radius = np.array([2,8])*RES, 1*RES
    obs2 = np.linalg.norm(pts-center, axis=1) < radius
    obs = np.logical_or(obs, obs2)
    
    # add circle 2
    center, radius = np.array([2,2])*RES, 1*RES
    obs2 = np.linalg.norm(pts-center, axis=1) < radius
    obs = np.logical_or(obs, obs2)

    # create border
    obs2 = np.zeros((height,width), dtype=bool)
    bw = int(0.1 *RES) # border width
    obs2[:,0:bw] = obs2[0:bw,:] = obs2[:,-bw:] = obs2[-bw:,:] = True
    obs = np.logical_or(obs, obs2.flatten())

    obs_int = obs.reshape(height, width).astype('uint8')
    
    # create clearance kernel
    clearance_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(CLEARANCE*2, CLEARANCE*2))
    # create obstacles with clearance
    obs_int = cv2.dilate(obs_int, clearance_kernel)*250 # 250 is color intensity
    
    # create robot footprint
    robot_shape = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(ROBOT_RADIUS*2, ROBOT_RADIUS*2))
    center = (robot_shape.shape[0]//2, robot_shape.shape[1]//2)

    conf = obs_int.copy()
    conf = cv2.dilate(conf, robot_shape)
    return conf, obs.reshape(height, width)

def run_application(start, goal):
    # find path
    planner = RrtApf(start, goal, conf, animate=False)

    if PLOT:
        ax.set_title("Searching Path")
        ticks = np.arange(0, 10*RES+1, RES)
        ax.set_xticks(ticks)
        ax.set_yticks(ticks)
        plt.pause(0.001)    # Necessary for updating title
    
    start_time = time.perf_counter()
    path, nodes = planner.find_path()
    exec_time = time.perf_counter() - start_time
    
    if PLOT:
        message = f"Path Found! in {round(exec_time, 2)} sec."
        ax.set_title(message)
        ax.plot(path[:,0],path[:,1], c='r')
        ax.plot(np.array(nodes)[:, 0], np.array(nodes)[:, 1], 'b.', markersize=2)
        print("Press 'q' to start following path")
        plt.show()

    path /= RES # rescale it to original resolution
    return path

def parse_args(argv):
    if len(argv) == 1:  # running with no arguments
        return (1*RES, 1*RES), (9*RES, 9*RES), 5, 10, 0.1, True
    elif len(argv) > 9: # Input from launch file
        argv = [float(i) for i in argv[:8]]
        start = (argv[0]*RES, argv[1]*RES)
        goal = (argv[3]*RES, argv[4]*RES)
        return start, goal, argv[5], argv[6], argv[7], True
    elif len(argv) == 9:  # running stand alone
        argv = [float(i) for i in argv[1:9]]
        return (argv[0]*RES, argv[1]*RES), \
               (argv[3]*RES, argv[4]*RES), argv[5], argv[6], argv[7], True
    print("Wrong input arguments")
    sys.exit()

# =============================================================================    
# Main logic
# =============================================================================    

def main(argv=[]):
    global width, height, conf, ax
    global RES, STEP_SIZE, ROBOT_RADIUS, CLEARANCE, GOAL_RADIUS, PLOT
    RES = 20  # parts per meter

    start, goal, rpm1, rpm2, clearance, PLOT = parse_args(argv)
    PLOT = True
    GOAL_RADIUS = round(0.25 *RES)
    ROBOT_RADIUS = round(0.2 *RES)
    CLEARANCE = round(clearance *RES)
    STEP_SIZE = round(1 *RES)

    # create exploration space
    width, height = 10*RES+1, 10*RES+1
    Y, X = np.mgrid[0:height, 0:width]
    pts = np.array([X.ravel(), Y.ravel()]).T

    # create obstacle map
    conf, obs = get_obstacle_map(pts)

    if check_if_invalid(round(start[0]), round(start[1])):
        print("Start position is not in configuration space. Exiting.")
        sys.exit()
    if check_if_invalid(round(goal[0]), round(goal[1])):
        print("Goal position is not in configuration space. Exiting.")
        sys.exit()
    
    if PLOT:
        fig, ax = plt.subplots()
        ax.axis('equal')
        # plt.subplots_adjust(bottom=0.15)
        ax.scatter(pts[obs.flatten(),0], pts[obs.flatten(),1], s=1, c='k')
    
    return run_application(start, goal)

if __name__ == "__main__":
    main(sys.argv)