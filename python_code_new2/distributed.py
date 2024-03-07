"""
This file contains a placeholder for the DistributedPlanningSolver class that can be used to implement distributed planning.

Code in this file is just provided as guidance, you are free to deviate from it.
"""

import time as timer
import numpy as np
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
from distributed_agent_class import DistributedAgent
from cbs import detect_collision, detect_collisions
from visualize import Animation

class DistributedPlanningSolver(object):
    """A distributed planner"""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """
        self.CPU_time = 0
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.heuristics = []

        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))


    def find_solution(self):
        start_time = timer.time()
        result = []
        self.CPU_time = timer.time() - start_time
        constraints = []
        # neighbours = []  ### zelf ingezet
        collision = []
        condition = True
        time = 0

        # Create agent objects with DistributedAgent class
        for i in range(self.num_of_agents):
            newAgent = DistributedAgent(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i)



        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i, constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

        while time < 30 and condition == True:
            radius = []
            neighbours = []
            # Create constraints corresponding with the environment (already done)
            # for i in range(self.num_of_agents):  # Find path for each agent
            #     if i == 0:
            #         constraints.append({'agent': i, 'loc': [self.goals[i]], 'timestep': 4, 'positive': False})
            #         radius = []  ### zelf ingezet
            #         print('constraints', constraints)
            #         # result.append(i)
            print('start_time', time)

            # Find neighboring agents within radius
            for i in range(self.num_of_agents):
                position = Animation.get_state(time, result[i])
                # radius.append([[position[0], position[1]],[position[0], position[1]+1]])
                radius.append([[position[0], position[1]], [position[0]+1, position[1]], [position[0]+2, position[1]], [position[0], position[1]+1], [position[0], position[1]+2], [position[0]-1, position[1]], [position[0]-2, position[1]], [position[0], position[1]-1], [position[0], position[1]-2], [position[0]+1, position[1]+1], [position[0]-1, position[1]-1], [position[0]+1, position[1]-1], [position[0]-1, position[1]+1]])
                print(position)
            for i in range(self.num_of_agents):
                for j in range(self.num_of_agents):
                    if i != j:
                        for t in range(len(radius[0])):
                            for k in range(len(radius[0])):
                                if radius[i][t] == radius[j][k]:
                                    neighbours.append([i,j])

            detected = list(set(map(lambda x: tuple(sorted(x)), neighbours)))
            print(time, 'detected neighbors', detected) # dit is de lijst met buren die elkaar zien (niet elkaar, maar dus elkaars radius zien)

            # Create definition which detects collision
            # Create definition which resolves collision by creating constraints for certain agents
            # Now calculate new path for agents which now have new constraints

            time += 1
            ## if all random time, condition is False
            # if time > 100:
            #     condition = False

            # Print final output
        print("\n Found a solution! \n")
        print(result)
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))  # Hint: think about how cost is defined in your implementation


        return result  # Hint: this should be the final result of the distributed planning (visualization is done after planning)
