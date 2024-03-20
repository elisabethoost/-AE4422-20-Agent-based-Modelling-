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
            # constraints = [{'agent': 4, 'loc': [(1, 4)], 'timestep': 3, 'positive': False}]
            # constraints = [{'agent': 4, 'loc': (1, 4), 'timestep': 3, 'positive': False}]
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i, constraints)
            if path is None:
                raise BaseException('No solutions')
            first_result.append(path)
        print('initial constraints', constraints)
        print('first result', first_result)
        intermediate_result = first_result.copy()

        while time < 30 and condition == True:
            radius = []
            neighbours = []

            for i in range(self.num_of_agents):
                for j in range(self.num_of_agents):
                    if self.goals[i] == intermediate_result[i][0]:
                        if i != j:
                            # for t in range(time+3):
                            for t in range(3):
                                constraints.append({'agent': j, 'loc': [self.goals[i]], 'timestep': t, 'positive': False})


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
            detected_ = list(set(map(lambda x: tuple(sorted(x)), neighbours)))
            print(time, 'detected neighbors', detected_) # dit is de lijst met buren die elkaar zien (niet elkaar, maar dus elkaars radius zien)
            combined = separate_list(detected_)
            # print('detected', detected_)
            close_agents = list(set(combined))
            # print('combined', close_agents)
            # Detect collisions
            # detected = detected_[::-1]
            detected = detected_
            possible_collision = 1

            for i in range(len(detected)):
                agent0, agent1 = detected[i]
                possible_collision, location, time_to_collision = detect_collision(agent0, agent1, intermediate_result)

                if possible_collision != 0: ### if there is a collision detected soon
                    print('agents', detected[i],'detect a collision', time_to_collision, 'seconds from now')
                    current_location = intermediate_result
                    result1, agent_changed, constr = avoid_collision(self.my_map, current_location, self.goals,
                                                                     self.heuristics, constraints, agent0, agent1,
                                                                     location,
                                                                     time_to_collision, time, possible_collision, close_agents)
                    constraints = constr
                    intermediate_result[agent_changed] = result1[agent_changed]

                # if possible_collision == 3:
                #     print('agents', detected[i],'detect a collision', time_to_collision, 'seconds from now')
                #     current_location = intermediate_result
                #     result1, agent_changed, constr = avoid_edge_collision(self.my_map, current_location, self.goals,
                #                                                      self.heuristics, constraints, agent0, agent1,
                #                                                      location,
                #                                                      time_to_collision, time, possible_collision)
                #     constraints = constr
                #     intermediate_result[agent_changed] = result1[agent_changed]

            for i in range(len(intermediate_result)):
                final_result[i].append(intermediate_result[i][0])  # Append a copy of the location
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
        # return first_result  # Hint: this should be the final result of the distributed planning (visualization is done after planning)


def detect_collision(agent1, agent2, result):
    agent1_path, agent2_path = result[agent1], result[agent2]
    # Calculate the maximum number of timesteps to consider
    max_steps = min(len(agent1_path), len(agent2_path), 3)
    collision = 0
    location_detected = []
    t_t = []
    for t in range(max_steps):
        if agent1_path[t] == agent2_path[t]:
            collision += 1
            location_detected.append(agent1_path[t])
            t_t.append(t)
    for t in range(max_steps-1):
        if agent1_path[t+1] == agent2_path[t] and agent1_path[t] == agent2_path[t+1]:
            collision += 3
            location_detected.append([agent1_path[t], agent2_path[t]])
            t_t.append(t)
    # if collision == 1:
        # print('One vertex collision detected', collision)
        # print('location detected', location_detected)
    # elif collision == 2:
        # print('Two vertex collisions detected', collision)
        # print('location detected', location_detected)
    # elif collision == 3:
    #     print('One edge collision detected', collision)
    #     print('location detected', location_detected)
    # elif collision == 4:
    #     print('One vertex and one edge collisions detected', collision)
    #     print('location detected', location_detected)
    # elif collision == 5:
    #     print('Two vertex and one edge collision detected', collision)
    #     print('location detected', location_detected)
    # elif collision >= 6:
    #     print('At least two edge collisions detected, or one edge and three vertex', collision)
    #     print('location detected', location_detected)
    return collision, location_detected, t_t


def avoid_collision(map_, current_loc, goal, heuris, const, agent1, agent2, loc_of_collision, t_o_collision, t, collision, close_ag):
    agents = [agent1, agent2]
    i, j = agents
    max_steps = min(len(current_loc[i]), len(current_loc[j]), 3)
    for t in range(max_steps):
        print('LOOP 1')
        const.append({'agent': j, 'loc': [current_loc[i][t]], 'timestep': t, 'positive': False})
        const.append({'agent': i, 'loc': [current_loc[j][t]], 'timestep': t, 'positive': False})
        if collision >= 3:
            print('LOOP 2')
            const.append({'agent': i, 'loc': [current_loc[j][t], current_loc[i][t]], 'timestep': t, 'positive': False})
            const.append({'agent': j, 'loc': [current_loc[i][t], current_loc[j][t]], 'timestep': t, 'positive': False})

    # for t in range(max_steps-1):
    #     const.append({'agent': i, 'loc': [current_loc[j][t+1], current_loc[j][t]], 'timestep': t, 'positive': False})
    #     const.append({'agent': j, 'loc': [current_loc[i][t+1], current_loc[i][t]], 'timestep': t, 'positive': False})

    print('agent', i,'is now at', current_loc[i][0], 'and will collide at', loc_of_collision)
    print('agent', j,'is now at', current_loc[j][0], 'and will collide at', loc_of_collision)
    # print(const)

    path_new_a = a_star(map_, current_loc[i][0], goal[i], heuris[i], i, const)
    path_new_b = a_star(map_, current_loc[j][0], goal[j], heuris[j], j, const)

    if len(path_new_a) > len(path_new_b):
        path_new = path_new_b
        print('new path for agent', j, 'is', path_new, 'to goal location', goal[j])
        changed_agent = j
    else:
        path_new = path_new_a
        changed_agent = i
        print('new path for agent', i, 'is', path_new, 'to goal location', goal[i])

    print('new path', path_new)
    if path_new is None:
        raise BaseException('No solutions')
    current_loc_trial = current_loc
    current_loc_trial[changed_agent] = path_new

    for i in close_ag:
        for j in close_ag:
            if i != j:
                collide, edge_location, time_point = detect_collision(i, j, current_loc_trial)
                if collide != 0:
                    print('Errors found in new path: new collisions between', i, 'and', j, 'at', edge_location, 'at time', time_point)
                    max_steps = min(len(current_loc[i]), len(current_loc[j]), 3)
                    for t in range(max_steps):
                        if collide == 1 or collide == 2:
                            const.append({'agent': j, 'loc': [current_loc[i][t]], 'timestep': t, 'positive': False})
                            const.append({'agent': i, 'loc': [current_loc[j][t]], 'timestep': t, 'positive': False})
                        elif collide == 3:
                            const.append({'agent': i, 'loc': [current_loc[j][t], current_loc[i][t]], 'timestep': t, 'positive': False})
                            const.append({'agent': j, 'loc': [current_loc[i][t], current_loc[j][t]], 'timestep': t, 'positive': False})
                        elif collide >= 4:
                            const.append({'agent': i, 'loc': [current_loc[j][t], current_loc[i][t]], 'timestep': t, 'positive': False})
                            const.append({'agent': j, 'loc': [current_loc[i][t], current_loc[j][t]], 'timestep': t, 'positive': False})
                            const.append({'agent': j, 'loc': [current_loc[i][t]], 'timestep': t, 'positive': False})
                            const.append({'agent': i, 'loc': [current_loc[j][t]], 'timestep': t, 'positive': False})
                    path_new_a = a_star(map_, current_loc[i][0], goal[i], heuris[i], i, const)
                    path_new_b = a_star(map_, current_loc[j][0], goal[j], heuris[j], j, const)
                    if len(path_new_a) > len(path_new_b):
                        path_new = path_new_b
                        print('new path for agent', j, 'is', path_new, 'to goal location', goal[j])
                        changed_agent = j
                    else:
                        path_new = path_new_a
                        changed_agent = i
                        print('new path for agent', i, 'is', path_new, 'to goal location', goal[i])

                    # current_loc_trial[changed_agent] = path_new

    current_loc[changed_agent] = path_new
    return current_loc, changed_agent, const


# def avoid_edge_collision(map_, current_loc, goal, heuris, const, agent1, agent2, loc_of_collision, t_o_collision, t, collision):
#     agents = [agent1, agent2]
#     i, j = agents
#     a = loc_of_collision[0][0]
#     b = loc_of_collision[0][1]
#     max_steps = min(len(current_loc[i]), len(current_loc[j]), 3)
#     for t in range(max_steps):
#         const.append({'agent': j, 'loc': [current_loc[i][t]], 'timestep': t, 'positive': False})
#         const.append({'agent': i, 'loc': [current_loc[j][t]], 'timestep': t, 'positive': False})
#     # for t in range(max_steps - 1):
#     #     const.append({'agent': i, 'loc': [current_loc[j][t + 1], current_loc[j][t]], 'timestep': t, 'positive': False})
#     #     const.append({'agent': j, 'loc': [current_loc[i][t + 1], current_loc[i][t]], 'timestep': t, 'positive': False})
#
#     path_new_a = a_star(map_, current_loc[i][0], goal[i], heuris[i], i, const)
#     path_new_b = a_star(map_, current_loc[j][0], goal[j], heuris[j], j, const)
#     if len(path_new_a) > len(path_new_b):
#         path_new = path_new_b
#         print('new path for agent', j, 'is', path_new, 'to goal location', goal[j])
#         changed_agent = j
#     else:
#         path_new = path_new_a
#         changed_agent = i
#         print('new path for agent', i, 'is', path_new, 'to goal location', goal[i])
#     # print('new path', path_new)
#
#     if path_new is None:
#         raise BaseException('No solutions')
#     current_loc_trial = current_loc
#     current_loc_trial[changed_agent] = path_new
#
#     collide, edge_location, time_point = detect_collision(i, j, current_loc_trial)
#     if collide == 1 or collide == 2:
#         for k in range(len(loc_of_collision)):
#             const.append({'agent': j, 'loc': [loc_of_collision[k]], 'timestep': t_o_collision[k], 'positive': False})
#             const.append({'agent': i, 'loc': [loc_of_collision[k]], 'timestep': t_o_collision[k], 'positive': False})
#         print('agent', i,'is now at', current_loc[i][0], 'and will collide at', loc_of_collision)
#         print('agent', j,'is now at', current_loc[j][0], 'and will collide at', loc_of_collision)
#         print(const)
#         path_new_a = a_star(map_, current_loc[i][0], goal[i], heuris[i], i, const)
#         path_new_b = a_star(map_, current_loc[j][0], goal[j], heuris[j], j, const)
#         if len(path_new_a) > len(path_new_b):
#             path_new = path_new_b
#             print('new path for agent', j, 'is', path_new, 'to goal location', goal[j])
#             changed_agent = j
#         else:
#             path_new = path_new_a
#             changed_agent = i
#             print('new path for agent', i, 'is', path_new, 'to goal location', goal[i])
#     if collide == 3:
#         a = edge_location[0][0]
#         b = edge_location[0][1]
#         print(a,b)
#
#         const.append({'agent': i, 'loc': [a,b], 'timestep': 1, 'positive': False})
#         const.append({'agent': j, 'loc': [b,a], 'timestep': 1, 'positive': False})
#         path_new_a = a_star(map_, current_loc[i][0], goal[i], heuris[i], i, const)
#         path_new_b = a_star(map_, current_loc[j][0], goal[j], heuris[j], j, const)
#         if len(path_new_a) > len(path_new_b):
#             path_new = path_new_b
#             print('new path for agent', j, 'is', path_new, 'to goal location', goal[j])
#             changed_agent = j
#         else:
#             path_new = path_new_a
#             changed_agent = i
#             print('new path for agent', i, 'is', path_new, 'to goal location', goal[i])
#
#     current_loc[changed_agent] = path_new
#     return current_loc, changed_agent, const



# def avoid_collision(map_, current_loc, goal, heuris, const, agent1, agent2, loc_of_collision, t_o_collision, t, collision):
#     agents = [agent1, agent2]
#     i, j = agents
#     for k in range(len(loc_of_collision)):
#         const.append({'agent': j, 'loc': [loc_of_collision[k]], 'timestep': t_o_collision[k], 'positive': False})
#         const.append({'agent': i, 'loc': [loc_of_collision[k]], 'timestep': t_o_collision[k], 'positive': False})
#     print('agent', i,'is now at', current_loc[i][0], 'and will collide at', loc_of_collision)
#     print('agent', j,'is now at', current_loc[j][0], 'and will collide at', loc_of_collision)
#     print(const)
#     path_new_a = a_star(map_, current_loc[i][0], goal[i], heuris[i], i, const)
#     path_new_b = a_star(map_, current_loc[j][0], goal[j], heuris[j], j, const)
#     if len(path_new_a) > len(path_new_b):
#         path_new = path_new_b
#         print('new path for agent', j, 'is', path_new, 'to goal location', goal[j])
#         changed_agent = j
#     else:
#         path_new = path_new_a
#         changed_agent = i
#         print('new path for agent', i, 'is', path_new, 'to goal location', goal[i])
#
#     print('new path', path_new)
#     if path_new is None:
#         raise BaseException('No solutions')
#     current_loc_trial = current_loc
#     current_loc_trial[changed_agent] = path_new
#
#
#     collide, edge_location, time_point = detect_collision(i, j, current_loc_trial)
#     if collide == 2:
#         a = edge_location[0][0]
#         b = edge_location[0][1]
#         print(a,b)
#         const.append({'agent': i, 'loc': [a,b], 'timestep': 1, 'positive': False})
#         const.append({'agent': j, 'loc': [b,a], 'timestep': 1, 'positive': False})
#         path_new_a = a_star(map_, current_loc[i][0], goal[i], heuris[i], i, const)
#         path_new_b = a_star(map_, current_loc[j][0], goal[j], heuris[j], j, const)
#         if len(path_new_a) > len(path_new_b):
#             path_new = path_new_b
#             print('new path for agent', j, 'is', path_new, 'to goal location', goal[j])
#             changed_agent = j
#         else:
#             path_new = path_new_a
#             changed_agent = i
#             print('new path for agent', i, 'is', path_new, 'to goal location', goal[i])
#
#     current_loc[changed_agent] = path_new
#     return current_loc, changed_agent, const

def separate_list(input_list):
    pairs = {}
    combined = []

    for pair in input_list:
        num1, num2 = pair
        if num1 not in pairs:
            pairs[num1] = set()
        if num2 not in pairs:
            pairs[num2] = set()
        pairs[num1].add(num2)
        pairs[num2].add(num1)

    for num, combined_nums in pairs.items():
        if len(combined_nums) > 1:
            combined.extend([num] + list(combined_nums))

    return combined


