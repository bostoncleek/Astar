"""
This file provides methods for Part A
"""

from grid import Grid
from path import Astar

import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
import numpy as np


def question_3():

    print("----------------------")
    print("Results for question 3")
    print("Naive A* on large grid")
    print("----------------------")

    # creat grid with cell size = 1
    cell_size = 1
    grid_large = Grid(cell_size)

    # load landmarks into grid
    landmarks = grid_large.get_landmarks()

    # create paths for A, B, C in question 3
    # to run the naive implementation
    astar_naive_A = Astar([0.5, -1.5], [0.5, 1.5], grid_large)
    astar_naive_B = Astar([4.5, 3.5], [4.5, -1.5], grid_large)
    astar_naive_C = Astar([-0.5, 5.5], [1.5, -3.5], grid_large)

    # run naive implementation
    astar_naive_A.naive()
    astar_naive_A.get_path()

    astar_naive_B.naive()
    astar_naive_B.get_path()

    astar_naive_C.naive()
    astar_naive_C.get_path()

    # results for naive approach to question 3 start and end configuration
    print("Path A [0.5, -1.5], [0.5, 1.5]")
    display_results(grid_large, cell_size, landmarks, astar_naive_A)

    print("Path B [4.5, 3.5], [4.5, -1.5]")
    display_results(grid_large, cell_size, landmarks, astar_naive_B)

    print("Path C [-0.5, 5.5], [1.5, -3.5]")
    display_results(grid_large, cell_size, landmarks, astar_naive_C)

def question_5():

    print("----------------------")
    print("Results for question 5")
    print("Online A* on large grid")
    print("----------------------")

    # creat grid with cell size = 1
    cell_size = 1
    grid_large = Grid(cell_size)

    # load landmarks into grid
    landmarks = grid_large.get_landmarks()

    # run the online implementation for the large grid
    # given the start and end defined in question 3
    astar_online_A = Astar([0.5, -1.5], [0.5, 1.5], grid_large)
    astar_online_B = Astar([4.5, 3.5], [4.5, -1.5], grid_large)
    astar_online_C = Astar([-0.5, 5.5], [1.5, -3.5], grid_large)

    # run online implementation
    astar_online_A.online()
    astar_online_A.get_path()

    astar_online_B.online()
    astar_online_B.get_path()

    astar_online_C.online()
    astar_online_C.get_path()

    # results for online approach to question 3 start and end configuration
    print("Path A [0.5, -1.5], [0.5, 1.5]")
    display_results(grid_large, cell_size, landmarks, astar_online_A)

    print("Path B [4.5, 3.5], [4.5, -1.5]")
    display_results(grid_large, cell_size, landmarks, astar_online_B)

    print("Path C [-0.5, 5.5], [1.5, -3.5]")
    display_results(grid_large, cell_size, landmarks, astar_online_C)


def question_7():
    print("----------------------")
    print("Results for question 7")
    print("Online A* on fine grid")
    print("----------------------")

    # creat grid with cell size = 0.1
    cell_size = 0.1
    grid_small = Grid(cell_size)

    # load landmarks into grid
    landmarks = grid_small.get_landmarks()

    # create paths for A, B, C in question 7 on small grid size
    astar_online_A = Astar([2.45, -3.55], [0.95, -1.55], grid_small)
    astar_online_B = Astar([4.95, -0.05], [2.45, 0.25], grid_small)
    astar_online_C = Astar([-0.55, 1.45], [1.95, 3.95], grid_small)

    # run online implementation
    astar_online_A.online()
    astar_online_A.get_path()

    astar_online_B.online()
    astar_online_B.get_path()

    astar_online_C.online()
    astar_online_C.get_path()

    # results for online approach to question 7 start and end configuration
    print("Path A [2.45, -3.55], [0.95, -1.55]")
    display_results(grid_small, cell_size, landmarks, astar_online_A)

    print("Path B [4.95, -0.05], [2.45, 0.25]")
    display_results(grid_small, cell_size, landmarks, astar_online_B)

    print("Path C [-0.55, 1.45], [1.95, 3.95]")
    display_results(grid_small, cell_size, landmarks, astar_online_C)


def display_results(grid, cell_size, landmarks, astar):

    fig, ax = plt.subplots(dpi=110)

    #loc = plticker.MultipleLocator(base=cell_size)
    loc = plticker.MultipleLocator(base=1)
    ax.xaxis.set_major_locator(loc)
    ax.yaxis.set_major_locator(loc)
    plt.grid(True, color='blue', linestyle='--')

    plt.imshow(grid.world.T, cmap=plt.cm.binary,
                    interpolation='none', origin='lower', extent=[-2, 5, -6, 6])


    plt.plot(astar.path_world[0], astar.path_world[1], color='red', linewidth=2)
    plt.scatter(astar.path_world[0], astar.path_world[1], color='red', s=20)


    plt.scatter(astar.start_world[0], astar.start_world[1], s=30, color='blue')
    #plt.text(astar.start_world[0], astar.start_world[1], 'S', fontsize =12,
                            #bbox=dict(facecolor='red', alpha=0.5))
    ax.annotate('Start', xy=(astar.start_world[0],astar.start_world[1]),
                            xytext=(0, -30), textcoords='offset points',
                            arrowprops=dict(arrowstyle="->"))
    plt.scatter(astar.goal_world[0], astar.goal_world[1], s=30, color='blue', label='Goal')
    ax.annotate('Goal', xy=(astar.goal_world[0],astar.goal_world[1]),
                            xytext=(0, 20), textcoords='offset points',
                            arrowprops=dict(arrowstyle="->"))


    plt.axis([-2, 5, -6, 6])
    plt.title("Grid World")
    plt.xlabel('x position (m)')
    plt.ylabel('y position (m)')

    plt.show()
