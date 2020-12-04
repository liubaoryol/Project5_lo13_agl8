#!/usr/bin/env python
from mpl_toolkits.mplot3d import Axes3D, art3d
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import sys
from math import sin, cos


def obstacleOnGraph(graph, file_name):
    ''' Draws the obstacles on a given graph
    Args:
        graph (plt.figure) 
        file_name (str)
    '''
    obstacles_temp = []
    for line in open(file_name):
        if len(line.rstrip()) > 0:
            obstacles_temp.append(line)
    obstacles = []
    for obstacle in obstacles_temp:
        points = obstacle.split(' ')
        obstacles.append([float(x) for x in points])
        
    # Adding obstacles to graph
    for obstacle in obstacles:
        graph.add_patch(patches.Rectangle((obstacle[0], obstacle[1]), obstacle[2], obstacle[3], fill='false', color='black'))
    
    plt.axis([-10, 10, -10, 10])
    # plt.plot(-1.3, -1.3, 'go') # source
    # plt.plot(1.2, 1.2, 'ro') # destination



def plotRobots(obstacle_path, path):
    '''
    Args:
        obstacle_path (str) a .txt file
        path (str) a .txt file
    
    '''
    lines = []
    for line in open(path):
        if len(line.rstrip()) > 0:
            lines.append(line)

    traject = []
    for line in lines[1:]:
        points = line.split(' ')
        if '\n' in points[-1]:
            traject.append([float(x) for x in points[:-1]])


    fig = plt.figure()
    graph = fig.gca()
    obstacleOnGraph(graph, obstacle_path)

    for i in range(0,4):
        # The first 8 points in `path` correspond to the robot's positions
        X = [p[i*2] for p in traject]
        Y = [p[i*2+1] for p in traject]
        graph.plot(X, Y)

        # Plotting the actual box
        boxVert = [[-0.3, -0.3], [0.3, -0.3], [0.3, 0.3], [-0.3, 0.3], [-0.3, -0.3]]

        for p in traject:
            x = []
            y = []
            for v in boxVert:
                x.append(v[0] * cos(p[i+8]) - v[1] * sin(p[i+8]) + p[i*2])
                y.append(v[0] * sin(p[i+8]) + v[1] * cos(p[i+8]) + p[i*2+1])
            graph.plot(x, y, 'k')

    plt.show()

def plotRoadmap(obstacle_path, roadmap):
    '''
    Plot the resulting PRM
    '''
    fig = plt.figure()
    graph = fig.gca()
    
    obstacleOnGraph(graph, obstacle_filename)

    X = [p[0] for p in path]
    Y = [p[1] for p in path]
    graph.plot(X, Y, 'go')
    
    plt.axis([-10, 10, -10, 10])
    plt.show()

