"""
This file provides methods for Part B
"""

from grid import Grid
from path import Astar
from navigate import Navigation

import numpy as np


def question_9():
    print("----------------------")
    print("Results for question 9")
    print("Driving paths from 7")
    print("----------------------")

    # creat grid with cell size = 0.1
    cell_size = 0.1
    grid = Grid(cell_size)

    # load landmarks into grid
    landmarks = grid.get_landmarks()

    # create paths for A, B, C in question 7 on small grid size
    astar_online_A = Astar([2.45, -3.55], [0.95, -1.55], grid)
    astar_online_B = Astar([4.95, -0.05], [2.45, 0.25], grid)
    astar_online_C = Astar([-0.55, 1.45], [1.95, 3.95], grid)

    # run online implementation
    astar_online_A.online()
    astar_online_A.get_path()

    astar_online_B.online()
    astar_online_B.get_path()

    astar_online_C.online()
    astar_online_C.get_path()

    # init pose
    pose_A = np.array([2.45, -3.55, -np.pi/2])
    pose_B = np.array([4.95, -0.05, -np.pi/2])
    pose_C = np.array([-0.55, 1.45, -np.pi/2])

    # init controls
    u = np.array([0, 0])

    # drive the paths
    print("Path A [2.45, -3.55], [0.95, -1.55]")
    nav_A = Navigation(grid, astar_online_A, False)
    nav_A.drive_paths(astar_online_A.path_world, pose_A, u)

    print("Path B [4.95, -0.05], [2.45, 0.25]")
    nav_B = Navigation(grid, astar_online_B, False)
    nav_B.drive_paths(astar_online_B.path_world, pose_B, u)

    print("Path C [-0.55, 1.45], [1.95, 3.95]")
    nav_C = Navigation(grid, astar_online_C, False)
    nav_C.drive_paths(astar_online_C.path_world, pose_C, u)



def question_10():
    print("----------------------")
    print("Results for question 10")
    print("Planning while driving paths from 7")
    print("----------------------")

    # creat grid with cell size = 0.1
    cell_size = 0.1
    grid = Grid(cell_size)

    # load landmarks into grid
    landmarks = grid.get_landmarks()

    # create paths for A, B, C in question 7 on small grid size
    astar_online_A = Astar([2.45, -3.55], [0.95, -1.55], grid)
    astar_online_B = Astar([4.95, -0.05], [2.45, 0.25], grid)
    astar_online_C = Astar([-0.55, 1.45], [1.95, 3.95], grid)

    # init pose
    pose_A = np.array([2.45, -3.55, -np.pi/2])
    pose_B = np.array([4.95, -0.05, -np.pi/2])
    pose_C = np.array([-0.55, 1.45, -np.pi/2])

    # init controls
    u = np.array([0, 0])

    # plan and drive
    print("Path A [2.45, -3.55], [0.95, -1.55]")
    nav_A = Navigation(grid, astar_online_A, True)
    nav_A.plan_drive(pose_A, u)

    print("Path B [4.95, -0.05], [2.45, 0.25]")
    nav_B = Navigation(grid, astar_online_B, True)
    nav_B.plan_drive(pose_B, u)

    print("Path C [-0.55, 1.45], [1.95, 3.95]")
    nav_C = Navigation(grid, astar_online_C, True)
    nav_C.plan_drive(pose_C, u)


def question_11():
    question_11_fine()
    question_11_coarse()


def question_11_fine():
    print("----------------------")
    print("Results for question 11")
    print("Planning while driving paths from 3")
    print("Fine Grid")
    print("----------------------")

    # creat grid with cell size = 0.1
    cell_size = 0.1
    grid_small = Grid(cell_size)

    # load landmarks into grid
    landmarks = grid_small.get_landmarks()

    # init pose
    pose_A = np.array([0.5, -1.5, -np.pi/2])
    pose_B = np.array([4.5, 3.5, -np.pi/2])
    pose_C = np.array([-0.5, 5.5, -np.pi/2])

    # init controls
    u = np.array([0, 0])


    # create paths for A, B, C in question 3 on small grid size
    astar_A_fine = Astar([0.5, -1.5], [0.5, 1.5], grid_small)
    astar_B_fine = Astar([4.5, 3.5], [4.5, -1.5], grid_small)
    astar_C_fine = Astar([-0.5, 5.5], [1.5, -3.5], grid_small)


    # plan and drive
    print("Path A [0.5, -1.5], [0.5, 1.5]")
    nav_A_fine = Navigation(grid_small, astar_A_fine, True)
    nav_A_fine.plan_drive(pose_A, u)

    print("Path B [4.5, 3.5], [4.5, -1.5]")
    nav_B_fine = Navigation(grid_small, astar_B_fine, True)
    nav_B_fine.plan_drive(pose_B, u)

    print("Path C [-0.5, 5.5], [1.5, -3.5]")
    nav_C_fine = Navigation(grid_small, astar_C_fine, True)
    nav_C_fine.plan_drive(pose_C, u)


def question_11_coarse():
    print("----------------------")
    print("Results for question 11")
    print("Planning while driving paths from 3")
    print("Coarse Grid")
    print("----------------------")

    # creat grid with cell size = 0.1
    cell_size = 1
    grid_large = Grid(cell_size)

    # load landmarks into grid
    landmarks = grid_large.get_landmarks()

    # init pose
    pose_A = np.array([0.5, -1.5, -np.pi/2])
    pose_B = np.array([4.5, 3.5, -np.pi/2])
    pose_C = np.array([-0.5, 5.5, -np.pi/2])

    # init controls
    u = np.array([0, 0])

    # create paths for A, B, C in question 3 on large grid size
    astar_A_coarse = Astar([0.5, -1.5], [0.5, 1.5], grid_large)
    astar_B_coarse = Astar([4.5, 3.5], [4.5, -1.5], grid_large)
    astar_C_coarse = Astar([-0.5, 5.5], [1.5, -3.5], grid_large)

    # plan and drive
    print("Path A [0.5, -1.5], [0.5, 1.5]")
    nav_A_coarse = Navigation(grid_large, astar_A_coarse, True)
    nav_A_coarse.plan_drive(pose_A, u)

    print("Path B [4.5, 3.5], [4.5, -1.5]")
    nav_B_coarse = Navigation(grid_large, astar_B_coarse, True)
    nav_B_coarse.plan_drive(pose_B, u)

    print("Path C [-0.5, 5.5], [1.5, -3.5]")
    nav_C_coarse = Navigation(grid_large, astar_C_coarse, True)
    nav_C_coarse.plan_drive(pose_C, u)


















#
