#! /usr/bin/python3
# refer to medium_bezier_matrix.py Omar Aflak
from math import factorial
import numpy as np
import matplotlib.pyplot as plt

def comb(n, k):
    return factorial(n) //  (factorial(k) * factorial(n - k))

def get_bezier_matrix(n):
    coef = [[comb(n, i) * comb(i, k) * (-1)**(i - k) for k in range(i + 1)] for i in range(n + 1)]
    # padding with zeros to create a square matrix
    print("coef:", coef)
    print("bezier matrix:", [row + [0] * (n + 1 - len(row)) for row in coef])
    print("row:", [row for row in coef])
    print("zero padding:", [1,1] + [0]*(3))
    return [row + [0] * (n + 1 - len(row)) for row in coef]

def evaluate_bezier(points, total):
    n = len(points) - 1
    print("n: ", n)
    T = lambda t: [t**i for i in range(n + 1)]
    M = get_bezier_matrix(n)
    return np.array([
        np.dot(np.dot(T(t), M), points)
        for t in np.linspace(0, 1, total)
    ])

# points = np.random.randn(4, 2)
points = np.array([[0, 0],[1, 0],[1, 1]])
print("points:", points)
x, y = points[:,0], points[:,1]
# print("x:", x)

new_points = evaluate_bezier(points, 11)
nx, ny = new_points[:,0], new_points[:,1]
print("new points:", new_points)

plt.figure(figsize=(11, 8))
plt.plot(nx, ny, 'b-')
plt.plot(x, y, 'ro')
plt.show()