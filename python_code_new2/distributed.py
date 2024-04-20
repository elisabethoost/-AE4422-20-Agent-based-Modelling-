"""
This is where most of our changes have been made. We have also changed a few functions in single_agent_planner, which are explained in the report.
Additionally, we added randon agent placement, which has been done in run_experiments.
We also added a pause button to the simulation for practicality, in visualize.
"""

import time as timer
import numpy as np
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost

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
        condition = True
        time = 0
        stop_time = 0

        # First, find path for each agent
        for i in range(self.num_of_agents):
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i, constraints, time)
            if path is None:
                raise BaseException('No solutions')
            first_result.append(path)

        #Then, save the original paths to intermediate_result.
        intermediate_result = first_result.copy()

        #The simulation is run for a maximum of 45 timesteps.
        while time < 45 and condition == True:
            #initiate lists - they are emptied every timestep.
            radius = []
            neighbours = []

            # Create radius for every agent.
            for i in range(self.num_of_agents):
                position = np.array(intermediate_result[i][0])
                radius.append([[position[0], position[1]], [position[0] + 1, position[1]],
                               [position[0] + 2, position[1]], [position[0], position[1] + 1],
                               [position[0], position[1] + 2], [position[0] - 1, position[1]],
                               [position[0] - 2, position[1]], [position[0], position[1] - 1],
                               [position[0], position[1] - 2], [position[0] + 1, position[1] + 1],
                               [position[0] - 1, position[1] - 1], [position[0] + 1, position[1] - 1],
                               [position[0] - 1, position[1] + 1], [position[0] + 3, position[1]],[position[0] - 3, position[1]],[position[0], position[1]+3],[position[0], position[1]-3], [position[0] + 4, position[1]],[position[0] - 4, position[1]],[position[0], position[1]+4],[position[0], position[1]-4]])
            # Find agents who see each other: neighbors
            for i in range(self.num_of_agents):
                for j in range(self.num_of_agents):
                    if i != j:
                        for t in range(13):
                            if radius[i][t] == radius[j][0]:
                                neighbours.append([i, j])
            # save neighbors, but filter list: [(1,2)(2,1)] = [(1,2)].
            detected = list(set(map(lambda x: tuple(sorted(x)), neighbours)))

            # check 3 times
            for q in range(0,3):
                #check for every couple whether they collide or not.
                for i in range(len(detected)):
                    agent0, agent1 = detected[i]
                    possible_collision = detect_collision(agent0, agent1, intermediate_result, self.goals)

                    #if they do not collide, possible_collision is zero, else, the collision should be avoided through rule based negotiation.
                    if possible_collision != 0:
                        current_location = intermediate_result
                        path_op_a, path_op_b, const_a, const_b = avoid_collision(self.my_map, current_location, self.goals,
                                                                         self.heuristics, constraints, agent0, agent1,
                                                                         time, possible_collision)

                        #path_op_a and path_op_b are the optional paths (re-routes) and const_a and const_b are the corresponding constraints.
                        #assign_routes checks which one of the agents goes around. So avoid collision is 'how', and assign routes is 'who'.
                        new_route_a, new_route_b, changed_agent_a, changed_agent_b, constr = assign_routes(self.my_map, self.heuristics, current_location, self.goals, constraints, const_a, const_b, path_op_a, path_op_b, agent0, agent1, time)

                        #now save the routes for a and b to the correct agents, and save the constraints that have to be added to the constraint table.
                        constraints.extend(constr)
                        intermediate_result[changed_agent_a] = new_route_a
                        intermediate_result[changed_agent_b] = new_route_b

            #The final result is the end result, with all the locations of all agents for all timesteps. Even if agents are already at their goal location, this location is added to the final result
            #Also remove the current locations from intermediate result. This is the paths the agents still have, (and thus not what they have travelled already)
            for i in range(len(intermediate_result)):
                final_result[i].append(intermediate_result[i][0])
            for i in range(len(first_result)):
                nu = intermediate_result[i][0]
                if len(intermediate_result[i]) <= 1:
                    intermediate_result[i].append(nu)
                else:
                    intermediate_result[i] = intermediate_result[i][1:]

            # created some stop conditions. If all agents are at their goal, simulation stops
            if stop_time == 0:
                if all(len(path) <= 1 or path[-2:] == [path[-1]]*2 for path in intermediate_result):
                    stop_time = time+1
            if time == stop_time and stop_time != 0:
                condition = False

            # update the CPU time and time
            self.timestep = time
            self.CPU_time = timer.time() - start_time
            time += 1
        # Print final output
        print("\n Found a solution! \n")
        print("first_result", first_result)
        print("final_result", final_result)
        self.CPU_time = timer.time() - start_time
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(final_result)))  # Hint: think about how cost is defined in your implementation
        return final_result
        # return first_result - uncomment this if you want to see what agents would do if they would follow their initially defined paths without conflict avoidance.


# This function detects whether 2 agents collide or not.
def detect_collision(agent1, agent2, result, goal):
    agent1_path, agent2_path = result[agent1], result[agent2]
    # it cannot look further in the future than the path lenghts are of the agents, therefore the future vision depends on this.
    max_steps = min(len(agent1_path), len(agent2_path), 5)

    #initialize collision to zero
    collision = 0

    # vertex collisions
    for t in range(max_steps):
        if agent1_path[t] == agent2_path[t]:
            collision += 1
    # edge collisions
    for t in range(max_steps-1):
        if agent1_path[t+1] == agent2_path[t] and agent1_path[t] == agent2_path[t+1]:
            collision += 3
    # if agent is at its own goal and other agent is there in 1 timestep too (future vision problem is avoided with this) (vertext collision)
    if len(agent2_path) > 1:
        if agent1_path[0] == goal[agent1] and agent2_path[1] == goal[agent1]:
            collision += 1
    # if other agent is at its own goal and other agent is there in 1 timestep too (future vision problem is avoided with this) (vertex collison)
    if len(agent1_path) > 1:
        if agent2_path[0] == goal[agent2] and agent1_path[1] == goal[agent2]:
            collision += 1
    # if collision = 1 or 2, there are 2 vertex collisions
    # if collision = 3, there is 1 edge collision or 3 vertex collisions (first is more likely)
    # if collision > 3, there is/are edge collisions and vertex collisions
    return collision


# here the optional paths for both agents are made, and the corresponding constraints.
# if agent 1 is rerouted and thus makes way for agent 2, the constraints due to agent b's position should be saved, because if agent 1 reroutes again due to another agent, it can cause problems.
def avoid_collision(map_, current_loc, goal, heuris, const, agent1, agent2, time, collision):
    agents = [agent1, agent2]
    const1 = const
    const2 = const
    i, j = agents
    max_steps = min(len(current_loc[i]), len(current_loc[j]), 5)

    #The following is based on the estimations:
    # if collision = 1 or 2, there are 2 vertex collisions
    # if collision = 3, there is 1 edge collision or 3 vertex collisions (first is more likely)
    # if collision > 3, there is/are edge collisions and vertex collisions

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
        # The reasoning for the constraints below is given in the paper, but in short
        # Look at how long the re-route would be, if the other agent who is at its goal location, will stay there the whole time.
        if current_loc[j][t] == goal[j]:
            const2.append({'agent': j, 'loc': [current_loc[i][t]], 'timestep': time + t, 'positive': False})
            for l in range(100):
                const1.append({'agent': i, 'loc': [goal[j]], 'timestep': time+t+l, 'positive': False})
        if current_loc[i][t] == goal[i]:
            const1.append({'agent': i, 'loc': [current_loc[j][t]], 'timestep': time + t, 'positive': False})
            for l in range(100):
                const2.append({'agent': j, 'loc': [goal[i]], 'timestep': time+t+l, 'positive': False})

    # based on the new constraints, new paths for both are found.
    path_new_a = a_star(map_, current_loc[i][0], goal[i], heuris[i], i, const1, time)
    path_new_b = a_star(map_, current_loc[j][0], goal[j], heuris[j], j, const2, time)
    return path_new_a, path_new_b, const1, const2


# the function below is almost the same as the one above, however it makes constraints based on the new paths, such that the right constraints can be saved when it is known which agent re-routes.
def avoid_collision_new_path(new_path_a, new_path_b, goal, const, agent1, agent2, time):
    agents = [agent1, agent2]
    const3 = const
    const4 = const
    i, j = agents
    if new_path_a is None or new_path_b is None:
        return const3, const4
    max_steps = min(len(new_path_a), len(new_path_b), 5)
    for t in range(max_steps):
        const4.append({'agent': j, 'loc': [new_path_a[t]], 'timestep': time+t, 'positive': False})
        const3.append({'agent': i, 'loc': [new_path_b[t]], 'timestep': time+t, 'positive': False})
        const3.append({'agent': i, 'loc': [new_path_b[t], new_path_a[t]], 'timestep': time+t, 'positive': False})
        const4.append({'agent': j, 'loc': [new_path_a[t], new_path_b[t]], 'timestep': time+t, 'positive': False})
        const4.append({'agent': j, 'loc': [new_path_a[t]], 'timestep': time+t, 'positive': False})
        const3.append({'agent': i, 'loc': [new_path_b[t]], 'timestep': time+t, 'positive': False})
        const3.append({'agent': i, 'loc': [new_path_b[t], new_path_a[t]], 'timestep': time+t, 'positive': False})
        const4.append({'agent': j, 'loc': [new_path_a[t], new_path_b[t]], 'timestep': time+t, 'positive': False})
        if new_path_b[t] == goal[j]:
            const4.append({'agent': j, 'loc': [new_path_a[t]], 'timestep': time + t, 'positive': False})
            for l in range(10):
                const3.append({'agent': i, 'loc': [goal[j]], 'timestep': time+t+l, 'positive': False})
        if new_path_a[t] == goal[i]:
            const3.append({'agent': i, 'loc': [new_path_b[t]], 'timestep': time + t, 'positive': False})
            for l in range(10):
                const4.append({'agent': j, 'loc': [goal[i]], 'timestep': time+t+l, 'positive': False})
    return const3, const4


# the function below decides which agent reroutes along the optional path, and which agent follows its original path
def assign_routes(map_, heuris, current_loc, goal, const, const1, const2, path_new_a, path_new_b, i, j, time):
    const_3, const_4 = avoid_collision_new_path(path_new_a, path_new_b, goal, const, i, j, time)
    # if agent is not at its goal, k_a and k_b are zero, else they are 2
    k_a = 0
    k_b = 0
    if current_loc[i][0] == goal[i]:
        k_a = 2
    if current_loc[j][0] == goal[j]:
        k_b = 2
    # making sure that optional_paht_lengths are never None
    # optional and additional, are two methods, and both work, and can be uncommented/commented
    optional_path_length_a = 100
    optional_path_length_b = 100
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

    # start negotionation:
    # if neigther agents can go out of the way, resolving the conflict is not possible.
    if path_new_a is None and path_new_b is None:
        raise BaseException('No solutions, both are none')

    # if only one agent can go another route or make way for the other agent, this agent is obligated to do so.
    elif path_new_a is None and path_new_b is not None:
        print(time, 'agent a has no path', i, j)
        path_b = path_new_b
        path_a = current_loc[i]
        new_const.extend(const2)
        new_const.extend(const_3)

    # if only one agent can go another route or make way for the other agent, this agent is obligated to do so.
    elif path_new_b is None and path_new_a is not None:
        print(time, 'agent b has no path', i, j)
        path_a = path_new_a
        path_b = current_loc[j]
        new_const.extend(const1)
        new_const.extend(const_4)

    # if both agents can go another route or make way for teh agent, they have to decide who
    elif path_new_a is not None and path_new_b is not None:
        print(time,'both agents have a path', i, j, k_a, k_b)
        # if they are both not at their goal location
        if k_a == 0 and k_b == 0:
            # then if the agent who is closest by its goal and can easily get there with the reroute, has to take the reroute
            if optional_path_length_a <= optional_path_length_b:
                path_a = path_new_a
                path_b = current_loc[j]
                new_const.extend(const1)
                new_const.extend(const_4)
            elif optional_path_length_a > optional_path_length_b:
                path_b = path_new_b
                path_a = current_loc[i]
                new_const.extend(const2)
                new_const.extend(const_3)

        # if agent a or b are its goal location
        if k_a == 2 and k_b == 0:
            print("AGENT A", i, "IS ALREADY AT GOAL")
            # then the agent's reroute, which causes the lowest additional time steps compared to the other, should make way
            # thus if agent a can take 2 steps to make way for b, and agent b can go around a but it takes 6 timesteps longer than the original route of b, then a has to take those 2 steps
            if additional_path_length_a <= additional_path_length_b:
            # if optional_path_length_a <= optional_path_length_b:
                path_a = path_new_a
                path_b = current_loc[j]
                new_const.extend(const1)
                new_const.extend(const_4)
            else:
                path_a = current_loc[i]
                path_b = path_new_b
                new_const.extend(const2)
                new_const.extend(const_3)
        if k_b == 2 and k_a == 0:
            print("AGENT B IS ALREADY AT GOAL", j)
            if additional_path_length_b <= additional_path_length_a:
            # if optional_path_length_b <= optional_path_length_a:
                path_b = path_new_b
                path_a = current_loc[i]
                new_const.extend(const2)
                new_const.extend(const_3)

            else:
                path_a = path_new_a
                path_b = current_loc[j]
                new_const.extend(const1)
                new_const.extend(const_4)
    if path_a is None or path_b is None:
        raise BaseException('No solutions, both are none')
    return path_a, path_b, i, j, new_const