"""
This is the main file for running all algorithms and displaying results.

All files written in python 3

See ReadMe.txt for help
"""

from partA import*
from partB import*

import numpy as np

def main():

    """
    File path to ds1 in top of grid.py
    """

    # Run all the requirements for part A
    ##############################
    # Question 3
    # runs naive A*
    question_3()

    ##############################
    # Question 5
    # runs online A*
    question_5()

    ##############################
    # Question 7
    # runs online A* on fine grid
    question_7()


    # Run all the requirements for part B
    ##############################
    # Question 9
    question_9()

    ##############################
    # Question 10
    question_10()

    ##############################
    # Question 1
    question_11()




if __name__ == "__main__":
    main()
