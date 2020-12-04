from turtle import *
import numpy as np
import sys
import math

# Constant parameters of window size and scale, and data filenames
PATH_FN = "path.txt"
OBSTACLES_FN = "obstacles.txt"
ENV_FN = "boundary.gif"
ENV_X = 500
ENV_Y = 500
ENV_SCALE = 500
TITLE = b"Environment Visualization"
PIXELS_PER_UNIT = ENV_SCALE/20

# Numpy loads data to a nested array
obstacles = np.loadtxt(OBSTACLES_FN)
path = np.loadtxt(PATH_FN)

s = Screen()
s.setup(width=ENV_X+50, height=ENV_Y+50)
s.delay(0)
s.tracer(0,0)
s.bgpic(ENV_FN)
s.title(TITLE)

t = Turtle() # The robot
t.speed(0)
rw = path[0][0]*PIXELS_PER_UNIT
rh = path[0][1]*PIXELS_PER_UNIT
# Draw square of dimensions specified in first line of path file
s.addshape("robot", ((-rw/2, -rh/2), (rw/2, -rh/2), (rw/2, rh/2), (-rw/2, rh/2)))
t.shape("robot")
t.pensize(3)
t.fillcolor("orange")
t.pencolor("black")

tobs = []
for i, obstacle in enumerate(obstacles[1:]): # Ignore "header" first line of obstacle file
    o = Turtle()
    o.seth(90) # Zero is pointing right, which is incorrect
    tobs.append(o)
    x = obstacle[0]*PIXELS_PER_UNIT
    y = obstacle[1]*PIXELS_PER_UNIT
    w = obstacle[2]*PIXELS_PER_UNIT
    h = obstacle[3]*PIXELS_PER_UNIT
    s.addshape(str(i), ((0, 0), (w, 0), (w, h), (0, h)))
    o.shape(str(i))
    o.fillcolor("red")
    o.pencolor("black")
    o.up()
    o.setpos(x, y)
    
trace_c = 1
def trace(): # Jump to the next state along path for a simple animation
    global trace_c
    if trace_c == 1:
        t.up()
        t.clearstamps()
    else:
        t.down()
    x = path[trace_c][0]*PIXELS_PER_UNIT
    y = path[trace_c][1]*PIXELS_PER_UNIT
    t.setpos(x, y)
    # Set heading (radians to degrees here)
    t.seth((path[trace_c][2] * 180 / math.pi) + 90) # rads to degs
    t.stamp()
    trace_c += 1
    if trace_c > len(path) - 1: # restart
        trace_c = 1
    s.update()
    s.ontimer(trace, 10)

trace()
s.mainloop()
