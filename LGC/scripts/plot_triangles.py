import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import sys

def read_triangles_from_txt(filename):
    triangles = []
    with open(filename, 'r') as file:
        for line in file:
            coords = list(map(float, line.strip().split()))
            triangle = {
                'vertex_A': np.array(coords[0:3]),
                'vertex_B': np.array(coords[3:6]),
                'vertex_C': np.array(coords[6:9])
            }
            triangles.append(triangle)
    return triangles

def plot_triangles(triangles):
    # 绘制三角形
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for triangle in triangles:
        vertices = np.array([triangle['vertex_A'], triangle['vertex_B'], triangle['vertex_C']])
        ax.add_collection3d(Poly3DCollection([vertices], facecolors='cyan', linewidths=1, edgecolors='r', alpha=.25))

     # 设置坐标轴范围
    ax.set_xlim([-20, 20])
    ax.set_ylim([-20, 20])
    ax.set_zlim([5, -5])

    # 设置坐标轴标签
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')

    plt.show()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python plot_triangles.py <triangles_txt>")
        sys.exit(1)

    triangles_txt = sys.argv[1]
    triangles = read_triangles_from_txt(triangles_txt)
    plot_triangles(triangles)
