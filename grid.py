"""
This file creates the grid

"""


import numpy as np

# file path to obstacles
landmark_file = "ds1_Landmark_Groundtruth.dat"

class Grid(object):

    def __init__(self, size):

        # bounds of world
        self.x_lower = -2
        self.x_upper = 5
        self.y_lower = -6
        self.y_upper = 6

        # size of each cell
        self.cell_size = size
        self.cell_x = np.arange(self.x_lower, self.x_upper, self.cell_size)
        self.cell_y = np.arange(self.y_lower, self.y_upper, self.cell_size)

        # matrix representation of world
        # init each element to 1 representing true cost
        self.world = np.ones([len(self.cell_x), len(self.cell_y)])

        # landmarks
        self.landmarks = []
        self.load_landmarks()

        # radius occupied by landmark
        self.R = 0.3

        # label cell containing landmark
        self.occupied_cells()



    def occupied_cells(self):
        """ labels the cells occupied by a landmark """

        for lm in self.landmarks:
            if self.cell_size < 1:
                # expand the range the landmark exists
                lm_x_range = np.arange(lm[0]-self.R, lm[0]+self.R, self.cell_size)
                lm_y_range = np.arange(lm[1]-self.R, lm[1]+self.R, self.cell_size)

                # loop through expanded ranges and compute grid positions
                for lm_x in lm_x_range:
                    for lm_y in lm_y_range:

                        row, col = self.cell_index([lm_x, lm_y])

                        # apply cost of occupied cell
                        try:
                            self.world[row][col] = 1000
                        except IndexError:
                            pass

            else:
                # apply cost of occupied cell
                row, col = self.cell_index(lm)
                try:
                    self.world[row][col] = 1000
                except IndexError:
                    pass

    def cell_index(self, coord):
        """ given coordinates in the world determines the index of a cell

        ith row is the x postion in the world
        jth column is the y position in the world

        Args:
            coordinates (list): x and y position

        Returns:
            list containing ith row and jth column
        """

        for x in range(len(self.cell_x)):
            if coord[0] >= self.cell_x[x] and coord[0] <= self.cell_x[x] + self.cell_size:
                i = x

        for y in range(len(self.cell_y)):
            if coord[1] >= self.cell_y[y] and coord[1] <= self.cell_y[y] + self.cell_size:
                j = y

        return [i, j]

    def world_coord(self, position, len):
        """ converts grid world positions to world coordinates

        Args:
            position (2D list): i and j positions in grid
            len (int): length of position array

        Returns:
            coord (np.array): shape nx2 first row x
            and second row y
        """

        if len > 1:
            x_world = []
            y_world = []

            for item in position:
                x_world.append(self.cell_size*item[0]+self.cell_size/2-2)
                y_world.append(self.cell_size*item[1]+self.cell_size/2-6)

        else:
            x_world = self.cell_size*position[0]+self.cell_size/2-2
            y_world = self.cell_size*position[1]+self.cell_size/2-6


        return np.array([x_world, y_world])

    def load_landmarks(self):
        """ creates a list containing coordinates of landmarks """

        file = open(landmark_file, "r")
        for line in file:
            if not line.startswith("#"):
                values = line.split()
                self.landmarks.append([float(values[1]), float(values[2])])
        file.close()


    def get_landmarks(self):
        """ returns list containing landmarks """
        lm_x = [x for x, y in self.landmarks]
        lm_y = [y for x, y in self.landmarks]
        return [lm_x, lm_y]













#
