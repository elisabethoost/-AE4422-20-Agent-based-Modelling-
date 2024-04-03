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
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i, constraints, time)
            if path is None:
                raise BaseException('No solutions')
            first_result.append(path)
        print('initial constraints', constraints)
        print('first result', first_result)
        intermediate_result = first_result.copy()

        while time < 45 and condition == True:
            print("time", time)
            radius = []
            neighbours = []

            for i in range(self.num_of_agents):
                for j in range(self.num_of_agents):
                    if self.goals[i] == intermediate_result[i][0]:
                        if i != j:
                            # for t in range(time+3):
                            for t in range(2):
                                constraints.append({'agent': j, 'loc': [self.goals[i]], 'timestep': t+time, 'positive': False})


            # Find neighboring agents within radius
            for i in range(self.num_of_agents):
                position = np.array(intermediate_result[i][0])
                # radius.append([[position[0], position[1]], [position[0] + 1, position[1]],
                #                [position[0] + 2, position[1]], [position[0], position[1] + 1],
                #                [position[0], position[1] + 2], [position[0] - 1, position[1]],
                #                [position[0] - 2, position[1]], [position[0], position[1] - 1],
                #                [position[0], position[1] - 2], [position[0] + 1, position[1] + 1],
                #                [position[0] - 1, position[1] - 1], [position[0] + 1, position[1] - 1],
                #                [position[0] - 1, position[1] + 1]])
                radius.append([[position[0], position[1]], [position[0] + 1, position[1]],
                               [position[0] + 2, position[1]], [position[0], position[1] + 1],
                               [position[0], position[1] + 2], [position[0] - 1, position[1]],
                               [position[0] - 2, position[1]], [position[0], position[1] - 1],
                               [position[0], position[1] - 2], [position[0] + 1, position[1] + 1],
                               [position[0] - 1, position[1] - 1], [position[0] + 1, position[1] - 1],
                               [position[0] - 1, position[1] + 1], [position[0] + 3, position[1]],[position[0] - 3, position[1]],[position[0], position[1]+3],[position[0], position[1]-3], [position[0] + 4, position[1]],[position[0] - 4, position[1]],[position[0], position[1]+4],[position[0], position[1]-4]])
            for i in range(self.num_of_agents):
                for j in range(self.num_of_agents):
                    if i != j:
                        for t in range(13):
                            if radius[i][t] == radius[j][0]:
                                neighbours.append([i, j])
            detected = list(set(map(lambda x: tuple(sorted(x)), neighbours)))

            # combined = separate_list(detected)
            # close_agents = list(set(combined))
            # detected = detected_

            for q in range(0,2):
            # for q in range(len(detected)):
                for i in range(len(detected)):
                    agent0, agent1 = detected[i]
                    possible_collision = detect_collision(agent0, agent1, intermediate_result, self.goals)
                    if possible_collision != 0:
                        current_location = intermediate_result
                        path_op_a, path_op_b, const_a, const_b = avoid_collision(self.my_map, current_location, self.goals,
                                                                         self.heuristics, constraints, agent0, agent1,
                                                                         time, possible_collision)

                        new_route_a, new_route_b, changed_agent_a, changed_agent_b, constr = assign_routes(self.my_map, self.heuristics, current_location, self.goals, const_a, const_b, path_op_a, path_op_b, agent0, agent1, time)

                        constraints.extend(constr)
                        intermediate_result[changed_agent_a] = new_route_a
                        intermediate_result[changed_agent_b] = new_route_b

            for i in range(len(intermediate_result)):
                final_result[i].append(intermediate_result[i][0])
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
            self.timestep = time
            time += 1

        # Print final output
        print("\n Found a solution! \n")
        print("first_result", first_result)
        print("final_result", final_result)
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(final_result)))  # Hint: think about how cost is defined in your implementation
        return final_result
        # return first_result


def detect_collision(agent1, agent2, result, goal):
    agent1_path, agent2_path = result[agent1], result[agent2]
    max_steps = min(len(agent1_path), len(agent2_path), 10)
    collision = 0
    for t in range(max_steps):
        if agent1_path[t] == agent2_path[t]:
            collision += 1
    for t in range(max_steps-1):
        if agent1_path[t+1] == agent2_path[t] and agent1_path[t] == agent2_path[t+1]:
            collision += 3
    if len(agent2_path) > 1:
        if agent1_path[0] == goal[agent1] and agent2_path[1] == goal[agent1]:
            collision += 1
    if len(agent1_path) > 1:
        if agent2_path[0] == goal[agent2] and agent1_path[1] == goal[agent2]:
            collision += 1
    return collision


def avoid_collision(map_, current_loc, goal, heuris, const, agent1, agent2, time, collision):
    agents = [agent1, agent2]
    const1 = const
    const2 = const
    i, j = agents
    max_steps = min(len(current_loc[i]), len(current_loc[j]), 5)
    for t in range(max_steps):
        if collision == 1 or collision == 2:
            const2.append({'agent': j, 'loc': [current_loc[i][t]], 'timestep': time+t, 'positive': False})
            const1.append({'agent': i, 'loc': [current_loc[j][t]], 'timestep': time+t, 'positive': False})
        if collision == 3:
            const1.append({'agent': i, 'loc': [current_loc[j][t], current_loc[i][t]], 'timestep': time+t, 'positive': False})
            const2.append({'agent': j, 'loc': [current_loc[i][t], current_loc[j][t]], 'timestep': time+t, 'positive': False})
        if collision > 3:
            const2.append({'agent': j, 'loc': [current_loc[i][t]], 'timestep': time+t, 'positive': False})
            const1.append({'agent': i, 'loc': [current_loc[j][t]], 'timestep': time+t, 'positive': False})
            const1.append({'agent': i, 'loc': [current_loc[j][t], current_loc[i][t]], 'timestep': time+t, 'positive': False})
            const2.append({'agent': j, 'loc': [current_loc[i][t], current_loc[j][t]], 'timestep': time+t, 'positive': False})
        if current_loc[j][t] == goal[j]:
            # for g in range(2):
            #     current_loc[j].extend(current_loc[j][t])
            const2.append({'agent': j, 'loc': [current_loc[i][t]], 'timestep': time + t, 'positive': False})
            for l in range(100):
                const1.append({'agent': i, 'loc': [goal[j]], 'timestep': time+t+l, 'positive': False})
        if current_loc[i][t] == goal[i]:
        #     for f in range(2):
        #         current_loc[i].extend(current_loc[i])
            const1.append({'agent': i, 'loc': [current_loc[j][t]], 'timestep': time + t, 'positive': False})
            for l in range(100):
                const2.append({'agent': j, 'loc': [goal[i]], 'timestep': time+t+l, 'positive': False})

    # if current_loc[i][0] == goal[i]:
    #     if current_loc[j][1] == goal[i] or current_loc[j][2]==goal[i]:
    #         # temp_goal = [(1, 1)]
    #         path_new_a = a_star(map_, current_loc[i][0], goal[i], heuris[i], i, const1, time)
    #         # path_new_a = a_star(map_, current_loc[i][0], temp_goal, heuris[i], i, const1, time)
    #
    #         # path_new_a = [(4,9), (3,9), (4,9)]
    # else:
    #     path_new_a = a_star(map_, current_loc[i][0], goal[i], heuris[i], i, const1, time)

    path_new_a = a_star(map_, current_loc[i][0], goal[i], heuris[i], i, const1, time)
    path_new_b = a_star(map_, current_loc[j][0], goal[j], heuris[j], j, const2, time)
    return path_new_a, path_new_b, const1, const2


def assign_routes(map_, heuris, current_loc, goal, const1, const2, path_new_a, path_new_b, i, j, time):
    # changed_agent = 100
    k_a = 0
    k_b = 0
    if current_loc[i][0] == goal[i]:
        k_a = 2
    if current_loc[j][0] == goal[j]:
        k_b = 2
    optional_path_length_a = 1000
    optional_path_length_b = 1000
    new_const = []
    path_a = None
    path_b = None
    additional_path_length_a = 100
    additional_path_length_b = 100
    if path_new_a is not None:
        optional_path_length_a = len(path_new_a) + len(current_loc[j])
        additional_path_length_a = len(path_new_a) - len(current_loc[i])
    if path_new_b is not None:
        optional_path_length_b = len(path_new_b) + len(current_loc[i])
        additional_path_length_b = len(path_new_b) - len(current_loc[j])
    if path_new_a is None and path_new_b is None:
        raise BaseException('No solutions, both are none')

    elif path_new_a is None and path_new_b is not None:
        print(time, 'agent a has no path', i, j)
        path_b = path_new_b
        path_a = current_loc[i]

    elif path_new_b is None and path_new_a is not None:
        print(time, 'agent b has no path', i, j)
        path_a = path_new_a
        path_b = current_loc[j]

    elif path_new_a is not None and path_new_b is not None:
        print(time,'both agents have a path', i, j, k_a, k_b)
        if k_a == 0 and k_b == 0:
            if optional_path_length_a <= optional_path_length_b:
                path_a = path_new_a
                path_b = current_loc[j]
                new_const.extend(const1)

            elif optional_path_length_a > optional_path_length_b:
                path_b = path_new_b
                path_a = current_loc[i]
                new_const.extend(const2)

        if k_a == 2 and k_b == 0:
            print("AGENT A", i, "IS ALREADY AT GOAL")
            # if optional_path_length_b > 50:
            # if additional_path_length_a <= 3:
            if additional_path_length_a <= additional_path_length_b:
            # if optional_path_length_a <= optional_path_length_b:
                path_a = path_new_a
                path_b = current_loc[j]
                new_const.extend(const1)
                print("first")
                print("path_new_a", path_new_a)
                print("currentloc_i", current_loc[i])
                print("path_new_b", path_new_b)
                print("currentloc_j", current_loc[j])
            else:
                path_a = current_loc[i]
                path_b = path_new_b
                new_const.extend(const2)
                print("second")
                print("path_new_a", path_new_a)
                print("currentloc_i", current_loc[i])
                print("path_new_b", path_new_b)
                print("currentloc_j", current_loc[j])
        if k_b == 2 and k_a == 0:
            print("AGENT B IS ALREADY AT GOAL", j)
            # if optional_path_length_a > 50:
            # if additional_path_length_b <= 3:
            if additional_path_length_b <= additional_path_length_a:
            # if optional_path_length_b <= optional_path_length_a:
                path_b = path_new_b
                path_a = current_loc[i]
                new_const.extend(const2)
            else:
                path_a = path_new_a
                path_b = current_loc[j]
                new_const.extend(const1)

    if path_a is None or path_b is None:
        raise BaseException('No solutions, both are none')

    return path_a, path_b, i, j, new_const


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
#
# def assign_routes(map_, heuris, current_loc, goal, const1, const2, path_new_a, path_new_b, i, j, time):
#     changed_agent = 100
#     k_a = 0
#     k_b = 0
#     if current_loc[i] == goal[i]:
#         k_a = 2
#     if current_loc[j] == goal[j]:
#         k_b = 2
#     optional_path_length_a = 1000
#     optional_path_length_b = 1000
#     new_const = []
#     path_a = None
#     path_b = None
#
#     if path_new_a is not None:
#         optional_path_length_a = len(path_new_a) + len(current_loc[j]) - k_b
#     if path_new_b is not None:
#         optional_path_length_b = len(path_new_b) + len(current_loc[i]) - k_a
#
#     if path_new_a is None and path_new_b is not None:
#         print(time, 'agent a has no path')
#         for k in range(len(path_new_b)):
#             const1.append({'agent': i, 'loc': [path_new_b[k]], 'timestep': time+k, 'positive': False})
#             const1.append({'agent': i, 'loc': [path_new_b[k],path_new_b[k-1]], 'timestep': time+k, 'positive': False})
#
#         if k_b == 2:
#             print("b is er al.")
#             for f in range(0,30):
#                 index_to_remove = next((index for index, item in enumerate(const1) if
#                                         item == {'agent': i, 'loc': [goal[j]], 'timestep': time+f, 'positive': False}),
#                                        None)
#                 if index_to_remove is not None:
#                     const2.pop(index_to_remove)
#                     print("Item found and removed.")
#                 else:
#                     print("Item not found in list.")
#
#         path_new_om = a_star(map_, current_loc[i][0], goal[i], heuris[i], i, const1, time)
#         # maken we een nieuw pad aan voor agent a
#         if path_new_om is not None:
#             # path_new_a = path_new_om
#             print("path_new_om_a is not None")
#             path_new = path_new_om
#             changed_agent = i
#             new_const.extend(const1)
#
#     elif path_new_b is None and path_new_a is not None:
#         print(time, 'agent b has no path')
#         for k in range(len(path_new_a)):
#             const2.append({'agent': j, 'loc': [path_new_a[k]], 'timestep': time+k, 'positive': False})
#             const2.append({'agent': j, 'loc': [path_new_a[k], path_new_a[k-1]], 'timestep': time+k, 'positive': False})
#
#         path_new_om = a_star(map_, current_loc[j][0], goal[j], heuris[j], j, const2, time)
#         # path_new_b = path_new_om
#         if path_new_b is not None:
#             changed_agent += 100
#             path_new = path_new_om
#             changed_agent = j
#
#     elif path_new_a is not None and path_new_b is not None:
#         print(time,'both agents have a path')
#         if k_a == 0 and k_b == 0:
#             if optional_path_length_a >= optional_path_length_b:
#                 path_new = path_new_a
#                 changed_agent = i
#             elif optional_path_length_a < optional_path_length_b:
#                 path_new = path_new_b
#                 changed_agent = j
#         elif k_a == 2 and k_b == 0:
#             if optional_path_length_b > 20:
#                 for k in range(len(path_new_b)):
#                     const2.append({'agent': i, 'loc': [path_new_a[k]], 'timestep': time, 'positive': False})
#                 path_new_om = a_star(map_, current_loc[j][0], goal[j], heuris[j], j, const2, time)
#                 path_b = path_new_om
#                 path_a = path_new_a
#                 new_const.extend(const2)
#
#         elif k_b == 2 and k_a == 0:
#             if optional_path_length_a > 20:
#                 for k in range(len(path_new_a)):
#                     const1.append({'agent': i, 'loc': [path_new_b[k]], 'timestep': time, 'positive': False})
#                 path_new_om = a_star(map_, current_loc[i][0], goal[i], heuris[i], i, const1, time)
#                 path_b = path_new_b
#                 path_a = path_new_om
#                 new_const.extend(const1)
#
#     # if path_new is None:
#     #     raise BaseException('No solutions')
#
#     if changed_agent == i:
#         path_a = path_new
#         path_b = current_loc[j]
#         new_const.extend(const1)
#
#     elif changed_agent == j:
#         path_a = current_loc[i]
#         path_b = path_new
#         new_const.extend(const2)
#
#     elif changed_agent >= 100:
#         # raise BaseException('No solutions', changed_agent)
#         if path_a is None:
#             path_a = current_loc[i]
#         if path_b is None:
#             path_b = current_loc[j]
#     # elif changed_agent == 100:
#     #     path_a = path_new_a
#     #     path_b = path_new_b
#     return path_a, path_b,i,j,new_const
