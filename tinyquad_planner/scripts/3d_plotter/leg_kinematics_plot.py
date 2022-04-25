#! /usr/bin/python3
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits import mplot3d
import math

# Setting 3 axes for the graph
plt.axes(projection='3d')

x_ = 1.0
y_ = 1.0
z_ = 1.0


# x, y, x
p1 = [0,0,0]
p2 = [1,0,1]
p3 = [0,1,1]

xs = [p1[0], p2[0], p1[0], p3[0]]
ys = [p1[1], p2[1], p1[1], p3[1]]

l1 = math.sqrt(x_**2 + z_**2)
l2 = math.sqrt(y_**2 + z_**2)
l3 = math.sqrt(x_**2 + y_**2 + z_**2)

# Define the z, y, x data
x = 1.0
y = 1.0
z = math.sqrt(x**2 + y**2)

# Plotting the line
plt.plot(xs, ys, 'r', linewidth=2)
plt.title('Plot a line in 3D')
plt.xlabel("x")
plt.ylabel("y")
# plt.zlabel("z")
plt.show()
