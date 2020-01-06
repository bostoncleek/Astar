"""
Plans and drives paths
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker

from control import Controller


class Navigation(object):
    """ provides path and controls from start to goal """

    def __init__(self, grid, astar, plan_and_drive):
        # other classes
        self.grid = grid
        self. astar = astar
        self.controller = Controller()

        # visual display
        self.fig, self.ax = plt.subplots()

        if plan_and_drive:
            self.display(False)
        else:
            self.display(True)

        self.l = 0.1
        self.wait = 0.01

        # threshold distance goal/waypoint
        self.threshold = 0.15


    def plan_drive(self, pose, u):
        """ plan and drive, the current path is not known a priori

        Args:
            pose (np.array): init pose
            u (np.array): init contols
        """

        # record current velocities
        self.controller.record_velocities(u)

        # final destination
        goal = self.astar.goal_world

        # get first waypoint
        waypoint = self.astar.drive_online()

        # waypoint list
        way_x = []
        way_y = []
        way_x.append(waypoint[0])
        way_y.append(waypoint[1])

        # keep navigating until goal is reached
        while not self.goal_reached(pose, goal):

            # plan from robots position
            grid_pos = self.grid.cell_index([pose[0], pose[1]])
            self.astar.curr_node.position = grid_pos

            # waypoint achieved
            if self.dist_to_waypoint(pose, waypoint) <= self.threshold:
                # plan to next waypoint
                waypoint = self.astar.drive_online()
                way_x.append(waypoint[0])
                way_y.append(waypoint[1])

            # update controls
            u = self.controller.controls(pose, waypoint)

            # update robots position
            pose = self.controller.update_pose(pose, u)

            # show waypoint
            plt.scatter(waypoint[0], waypoint[1], color='red', linewidth=1)
            plt.plot(way_x, way_y,  color='red', linewidth=1)

            # draw robot
            plt.arrow(pose[0], pose[1], self.l*np.cos(pose[2]), self.l*np.sin(pose[2]),
                                head_width=0.2, color='yellow', ec='k')
            plt.scatter(pose[0], pose[1], s=90, color='purple', alpha=0.2)
            plt.pause(self.wait)

        plt.show()



    def drive_paths(self, path, pose, u):
        """ given a path the robot will drive it

        Args:
            path (np.array): waypoints in world, first row is x second in y
            pose (np.array): init pose
            u (np.array): init contols
        """

        # record current velocities
        self.controller.record_velocities(u)

        # index 1 contains first waypoint
        i = 1
        waypoint = [path[0][i], path[1][i]]

        # last waypoint if the goal
        goal = [path[0][path.shape[1]-1], path[1][path.shape[1]-1]]

        # keep navigating until goal is reached
        while not self.goal_reached(pose, goal):

            # waypoint achieved
            if self.dist_to_waypoint(pose, waypoint) <= self.threshold:
                #print("Waypoint: ", i, "reached!")
                i += 1
                waypoint = [path[0][i], path[1][i]]

            # update controls
            u = self.controller.controls(pose, waypoint)

            # update robots position
            pose = self.controller.update_pose(pose, u)

            # draw robot
            plt.arrow(pose[0], pose[1], self.l*np.cos(pose[2]), self.l*np.sin(pose[2]),
                                head_width=0.2, color='yellow', ec='k')
            plt.scatter(pose[0], pose[1], s=90, color='purple', alpha=0.2)
            plt.pause(self.wait)

        plt.show()


    def goal_reached(self, pose, goal):
        """ returns true if goal reached """
        if self.dist_to_waypoint(pose, goal) <= self.threshold:
            print("Goal reached by driving!")
            return True

        return False


    def dist_to_waypoint(self, pose, waypoint):
        """ distance from robot to waypoint """
        d = np.sqrt((pose[0]-waypoint[0])**2 + (pose[1]-waypoint[1])**2)
        return d


    def display(self, given_path=True):
        """ provides a visual display """

        #loc = plticker.MultipleLocator(base=self.grid.cell_size)
        loc = plticker.MultipleLocator(base=1)
        self.ax.xaxis.set_major_locator(loc)
        self.ax.yaxis.set_major_locator(loc)
        self.ax.grid(which='major', axis='both')
        plt.grid(True, color='blue', linestyle='--')

        plt.imshow(self.grid.world.T, cmap=plt.cm.binary,
                        interpolation='none', origin='lower', extent=[-2, 5, -6, 6])


        if given_path:
            plt.plot(self.astar.path_world[0], self.astar.path_world[1], color='red', linewidth=1)
            plt.scatter(self.astar.path_world[0], self.astar.path_world[1], color='red', s=10)


        plt.scatter(self.astar.start_world[0], self.astar.start_world[1], s=50, color='blue')
        self.ax.annotate('Start', xy=(self.astar.start_world[0], self.astar.start_world[1]),
                                xytext=(30, -20), textcoords='offset points',
                                arrowprops=dict(arrowstyle="->"))
        plt.scatter(self.astar.goal_world[0], self.astar.goal_world[1], s=50, color='blue', label='Goal')
        self.ax.annotate('Goal', xy=(self.astar.goal_world[0], self.astar.goal_world[1]),
                                xytext=(-10, 20), textcoords='offset points',
                                arrowprops=dict(arrowstyle="->"))

        plt.axis([-2, 5, -6, 6])
        plt.title("Grid World")
        plt.xlabel('x position (m)')
        plt.ylabel('y position (m)')














































#
