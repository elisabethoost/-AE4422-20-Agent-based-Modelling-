import time as timer
from single_agent_planner import compute_heuristics, a_star_one, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        result1 = []
        constraints = []

        for i in range(self.num_of_agents):  # Find path for each agent

            path = a_star_one(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            print(constraints)

            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            for j in range(self.num_of_agents):
                for t in range(len(path)): #t=0,1,2,3,4,5; t=0,1,2
                    if j > i:
                        # constraints.append({'agent': j, 'loc': path[3], 'timestep': 3, 'positive': False})

                        constraints.append( #vertex constraints
                            {
                                'agent': j,
                                'loc': [path[t]],
                                'timestep': t,
                                'positive': False
                            })
                        #edge constraints
                        constraints.append(
                            {
                                'agent': j,
                                'loc': [path[t], path[t-1]], #first location: where it is going, second timestep: curr loc, where it was
                                'timestep': t,
                                'positive': False
                            }
                        )
                # apply vertex constraint for finished agents. 'start_time' is when the constraint should apply
                constraints.append(
                    {
                        'agent': j,
                        'loc': [path[-1]],
                        'timestep': -1,
                        'start_time': len(path) - 1,
                        'positive': False
                    }
                    )

            # print(constraints)
            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches


            ##############################

        self.CPU_time = timer.time() - start_time


        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print('result', result)
        return result
