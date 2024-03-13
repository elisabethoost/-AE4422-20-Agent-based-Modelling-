"""
This file contains a placeholder for the DistributedPlanningSolver class that can be used to implement distributed planning.

Code in this file is just provided as guidance, you are free to deviate from it.
"""

import time as timer
import numpy as np
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
from distributed_agent_class import DistributedAgent
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
        first_result = []
        final_result = [[] for _ in range(self.num_of_agents)]

        self.CPU_time = timer.time() - start_time
        constraints = []
        collision = []
        condition = True
        time = 0
        stop_time = 0

        # Create agent objects with DistributedAgent class
        for i in range(self.num_of_agents):
            newAgent = DistributedAgent(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i)

        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i, constraints)
            if path is None:
                raise BaseException('No solutions')
            first_result.append(path)
        print('first result', first_result)

        intermediate_result = first_result.copy()

        while time < 30 and condition == True:
            radius = []
            neighbours = []

            # Find neighboring agents within radius
            for i in range(self.num_of_agents):
                position = np.array(intermediate_result[i][0])
                radius.append([[position[0], position[1]], [position[0] + 1, position[1]],
                               [position[0] + 2, position[1]], [position[0], position[1] + 1],
                               [position[0], position[1] + 2], [position[0] - 1, position[1]],
                               [position[0] - 2, position[1]], [position[0], position[1] - 1],
                               [position[0], position[1] - 2], [position[0] + 1, position[1] + 1],
                               [position[0] - 1, position[1] - 1], [position[0] + 1, position[1] - 1],
                               [position[0] - 1, position[1] + 1]])
            for i in range(self.num_of_agents):
                for j in range(self.num_of_agents):
                    if i != j:
                        for t in range(13):
                            if radius[i][t] == radius[j][0]:
                                neighbours.append([i, j])
            # for i in range(self.num_of_agents):
            #     for j in range(self.num_of_agents):
            #         if i != j:
            #             if any(radius[i]) in radius[j][0]:
            #                 neighbours.append([i, j])
            detected = list(set(map(lambda x: tuple(sorted(x)), neighbours)))
            print(time, 'detected neighbors', detected) # dit is de lijst met buren die elkaar zien (niet elkaar, maar dus elkaars radius zien)
            # Detect collisions
            for i in range(len(detected)):
                agent0, agent1 = detected[i]
                possible_collision, location, time_to_collision = detect_collision(agent0, agent1, intermediate_result)
                ##detect collision returns:
                ## possible collision == 1 in collision is detected
                ## location is the location of the collision if it is detected, otherwise it is empty
                ## time_to_collision is the time to collision if it is detected, otherwise it is empty

                if possible_collision == 1: ### if there is a collision detected soon
                    print('agents', detected[i],'detect a collision', time_to_collision, 'seconds from now')
                    current_location = intermediate_result
                    for i in range(len(location)):
                        result1, agent_changed, constr = avoid_collision(self.my_map, current_location, self.goals,
                                                                         self.heuristics, constraints, agent0, agent1,
                                                                         intermediate_result, location[i],
                                                                         time_to_collision[i])
                        # constraints.update(constr)
                        constraints = constr
                        intermediate_result[agent_changed] = result1[agent_changed]
            # Create definition which resolves collision by creating constraints for certain agents
            # Now calculate new path for agents which now have new constraints
            for i in range(len(intermediate_result)):
                final_result[i].append(intermediate_result[i][0])  # Append a copy of the location

            # intermediate_result = [pathway[1:] for pathway in intermediate_result]
            for i in range(len(first_result)):
                nu = intermediate_result[i][0]
                if len(intermediate_result[i]) <= 1:
                    intermediate_result[i].append(nu)
                else:
                    intermediate_result[i] = intermediate_result[i][1:]

            if stop_time == 0:
                if all(len(path) <= 1 or path[-2:] == [path[-1]]*2 for path in intermediate_result):
                    stop_time = time+1
            if time == stop_time and stop_time != 0:
                condition = False

            time += 1

        # Print final output
        print("\n Found a solution! \n")
        print("first_result", first_result)
        print("final_result", final_result)
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(final_result)))  # Hint: think about how cost is defined in your implementation
        return final_result  # Hint: this should be the final result of the distributed planning (visualization is done after planning)


def detect_collision(agent1, agent2, result):
    agent1_path, agent2_path = result[agent1], result[agent2]
    # agent2_path = result[agent2]
    # Calculate the maximum number of timesteps to consider

    # max_steps = min(len(agent1_path) - times, len(agent2_path) - times, 3)
    # max_steps = min(len(agent1_path)-1, len(agent2_path)-1, 3)
    max_steps = min(len(agent1_path), len(agent2_path), 3)

    # max_steps = min(len(agent1_path) - timestep, 3)
    collision = 0
    location_detected = []
    t_t = []
    # for t in range(times, times + max_steps):
    # print('agent path', agent1, agent1_path)
    # print('agent path', agent2, agent2_path)
    for t in range(max_steps):
        if agent1_path[t] == agent2_path[t]:
            collision = 1
            location_detected.append(agent1_path[t])
            t_t.append(t)
    return collision, location_detected, t_t


def avoid_collision(map_, current_loc, goal, heuris, const, agent1, agent2, result, loc_of_collision, t_o_collision):
    agents = [agent1, agent2]
    # np.random.shuffle(agents)
    i, j = agents
    const.append({'agent': j, 'loc': loc_of_collision, 'timestep': t_o_collision, 'positive': False})
    print('constraint added at timestep', t_o_collision, 'for agent', j, 'at location', loc_of_collision, 'to avoid collision with agent', i)
    print('agent is now at', current_loc[j][0], 'and will collide at', loc_of_collision)
    print(const)

    path_new = a_star(map_, current_loc[j][0], goal[j], heuris[j], j, const)
    if path_new is None:
        raise BaseException('No solutions')
    print('new path for agent', j, 'is', path_new, 'to goal location', goal[j])
    # print('current location of agent', j, 'is', current_loc[j][0])
    # result[j] = [(1, 5), (2, 5), (2, 4), (2, 3), (3, 3)]
    # result[j] = [(1, 5), (2, 5), (1, 5), (1, 4), (1, 3), (2, 3), (3, 3)]
    # result[j] = [(1, 5), (1, 5), (1, 4), (1, 3), (2, 3), (3, 3)]

    result[j] = path_new
    return result, j, const

