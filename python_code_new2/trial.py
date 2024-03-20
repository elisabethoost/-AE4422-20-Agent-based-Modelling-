
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

        intermediate_result = result
        while time < 30 and condition == True:
            radius = []
            neighbours = []

            # Find neighboring agents within radius
            for i in range(self.num_of_agents):
                position = Animation.get_state(time, result[i])
                # radius.append([[position[0], position[1]],[position[0], position[1]+1]])
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
                        for t in range(len(radius[0])):
                            if radius[i][t] == radius[j][0]:
                                neighbours.append([i, j])

            detected = list(set(map(lambda x: tuple(sorted(x)), neighbours)))
            print(time, 'detected neighbors',
                  detected)  # dit is de lijst met buren die elkaar zien (niet elkaar, maar dus elkaars radius zien)
            # Detect collisions
            for i in range(len(detected)):
                agent0, agent1 = detected[i]
                possible_collision, location, time_of_collision = detect_collision(agent0, agent1, time, result)

                if possible_collision == 1:  ### if there is a collision soon
                    print('agents', detected[i], 'detect a collision in ', time_of_collision, 'seconds')
                    result1, agent_changed, constraints = avoid_collision(self.my_map, self.starts,
                                                                          self.goals, self.heuristics, constraints,
                                                                          agent0, agent1, result, location,
                                                                          time + time_of_collision)
                    result[agent_changed] = result1[agent_changed]
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
        print("Sum of costs:    {}".format(
            get_sum_of_cost(result)))  # Hint: think about how cost is defined in your implementation

        return result  # Hint: this should be the final result of the distributed planning (visualization is done after planning)


def detect_collision(agent1, agent2, timestep, result):
    agent1_path = result[agent1]
    agent2_path = result[agent2]
    # Calculate the maximum number of timesteps to consider
    max_steps = min(len(agent1_path) - timestep, len(agent2_path) - timestep, 3)
    a = 0
    b = 0
    t_t = 0
    for t in range(timestep, timestep + max_steps):
        if agent1_path[t] == agent2_path[t]:
            a = 1
            b = agent1_path[t]
            t_t = t
    return a, b, t_t


def avoid_collision(map, start, goal, heuris, const, agent1, agent2, result, loc, t_o_collision):
    agents = [agent1, agent2]
    np.random.shuffle(agents)
    i, j = agents
    const.append({'agent': j, 'loc': loc, 'timestep': t_o_collision, 'positive': False})
    print('constraint added at timestep', t_o_collision, 'for agent', j, 'at location', loc,
          'to avoid collision with agent', i)

    # const.append({'agent': j, 'loc': [result[i][timestep+1]], 'timestep': timestep+1, 'positive': False})
    # const.append({'agent': j, 'loc': [result[i][timestep+2]], 'timestep': timestep+2, 'positive': False})

    # for g in range(num_agent):  # Find path for each agent
    path_new = a_star(map, start[j], goal[j], heuris[j], j, const)
    if path_new is None:
        raise BaseException('No solutions')
    result[j] = path_new
    return result, j, const

