"""
Main file to run experiments and show animation.

Note: To make the animation work in Spyder you should set graphics backend to 'Automatic' (Preferences > Graphics > Graphics Backend).
"""

#!/usr/bin/python
import argparse
import random
import glob
from pathlib import Path
from cbs import CBSSolver
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from distributed import DistributedPlanningSolver # Placeholder for Distributed Planning
from visualize import Animation
from single_agent_planner import get_sum_of_cost

SOLVER = "Distributed"
# SOLVER = "Prioritized"
def print_mapf_instance(my_map, starts, goals):
    """
    Prints start location and goal location of all agents, using @ for an obstacle, . for a open cell, and
    a number for the start location of each agent.

    Example:
        @ @ @ @ @ @ @
        @ 0 1 . . . @
        @ @ @ . @ @ @
        @ @ @ @ @ @ @
    """
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    """
    See docstring print_mapf_instance function above.
    """
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)

## Uncomment this function if: You want DO NOT want random placement of agents. Comment the function below this one if you uncomment this one.
def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')

    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)

    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    #agents
    line = f.readline()
    num_agents = int(line)
    #agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, starts, goals


## Uncomment this function if: You want want random placement of agents. Comment the function above this one if you uncomment this one.
# def import_mapf_instance(filename):
#     f = Path(filename)
#     if not f.is_file():
#         raise BaseException(filename + " does not exist.")
#     f = open(filename, 'r')
#
#     # First line: #rows #columns
#     line = f.readline()
#     rows, columns = [int(x) for x in line.split()]
#
#     # Rows and Columns
#     rows = int(rows)
#     columns = int(columns)
#
#     # Map
#     my_map = []
#     for r in range(rows):
#         line = f.readline()
#         my_map.append([])
#         for cell in line:
#             if cell == '@':
#                 my_map[-1].append(True)
#             elif cell == '.':
#                 my_map[-1].append(False)
#
#     # Close the file
#     f.close()
#
#     # Generate random number of agents (between 1 and 5): uncomment line below and comment the one below that.
#     # num_agents = random.randint(1, 5)
#     num_agents = 4
#     # Generate random start and goal locations for each agent
#     starts = []
#     goals = []
#     for _ in range(num_agents):
#         # Generate random start and goal locations
#         start_x, start_y, goal_x, goal_y = -1, -1, -1, -1
#         while True:
#             start_x = random.randint(0, rows - 1)
#             start_y = random.randint(0, columns - 1)
#             goal_x = random.randint(0, rows - 1)
#             goal_y = random.randint(0, columns - 1)
#             if (start_x, start_y) not in starts and (goal_x, goal_y) not in goals \
#                     and not my_map[start_x][start_y] and not my_map[goal_x][goal_y]:
#                 break
#         # Append start and goal to the list
#         starts.append((start_x, start_y))
#         goals.append((goal_x, goal_y))
#     return my_map, starts, goals


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,Independent,Prioritized}), defaults to ' + str(SOLVER))

    args = parser.parse_args()
    # Hint: Command line options can be added in Spyder by pressing CTRL + F6 > Command line options.
    # In PyCharm, they can be added as parameters in the configuration.

    result_file = open("results.csv", "w", buffering=1)

    for file in sorted(glob.glob(args.instance)):

        print("Import an instance")
        my_map, starts, goals = import_mapf_instance(file)
        print_mapf_instance(my_map, starts, goals)

        if args.solver == "CBS":
            print("Run CBS")
            cbs = CBSSolver(my_map, starts, goals)
            paths = cbs.find_solution(args.disjoint)
        elif args.solver == "Independent":
            print("Run Independent")
            solver = IndependentSolver(my_map, starts, goals)
            paths = solver.find_solution()
        elif args.solver == "Prioritized":
            print("Run Prioritized")
            solver = PrioritizedPlanningSolver(my_map, starts, goals)
            paths = solver.find_solution()
        elif args.solver == "Distributed":  # Wrapper of distributed planning solver class
            print("Run Distributed Planning")
            solver = DistributedPlanningSolver(my_map, starts, goals) #, ...) ## TODO: add your own distributed planning implementation here.
            paths = solver.find_solution()
        else:
            raise RuntimeError("Unknown solver!")

        cost = get_sum_of_cost(paths)
        result_file.write("{},{}\n".format(file, cost))


        if not args.batch:
            print("Test paths on a simulation")
            animation = Animation(my_map, starts, goals, paths)
            # animation.save("output.mp4", 1.0) # install ffmpeg package to use this option
            animation.show()
    result_file.close()
