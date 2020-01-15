import numpy as np
from tool.maze import Cell
from tool.maze import Maze
from tool.symbol import SymbolArray
from pysmt.shortcuts import *

class Robot_Plan:
    def __init__(self, num_robot, maze, max_time, use_wall=True):
        self.num_robot = num_robot
        self.num_position_x = maze.nx
        self.num_position_y = maze.ny
        self.use_wall = use_wall
        self.max_time_step = max_time

        self.init_time_step = 0
        self.init_robot_loc = (0,0)
        # self.goal_time_steps = [max_time]
        self.goal_time_steps = [i for i in range(1, self.max_time_step+1)]
        self.goal_robot_loc = (self.num_position_x-1, self.num_position_y-1)

        self.at = SymbolArray("at", shape=(self.num_robot, self.num_position_x, self.num_position_y, self.max_time_step+1))
        self.move = SymbolArray("move", shape=(num_robot, self.num_position_x, self.num_position_y, self.num_position_x, self.num_position_y, self.max_time_step))
        self.maze = maze
        self.solver = None

    def init_solver(self):
        self.formula = self.encode_constraints()
        self.solver = Solver()
        self.solver.add_assertion(self.formula)

    def get_schema(self, action, para):
        assert action in [self.move]
        pre = None
        add = None
        dele = None
        if action is self.move:
            state = self.at
            assert len(para) == 6
            robot, init_loc_x, init_loc_y, tar_loc_x, tar_loc_y, time_step = para
            pre = state[robot, init_loc_x, init_loc_y, time_step]
            add = state[robot, tar_loc_x, tar_loc_y, time_step + 1]
            dele = state[robot, init_loc_x, init_loc_y, time_step + 1]
        return pre, add, dele

    def block_by_wall(self, x, y, tar_x, tar_y):
        assert x == tar_x or y == tar_y
        if x == tar_x:
            assert abs(tar_y - y) == 1
            direction = "S" if tar_y - y == 1 else "N"
        else:
            assert abs(tar_x - x) == 1
            direction = "E" if tar_x - x == 1 else "W"
        reverse_direction = Cell.wall_pairs[direction]
        if tar_x >= 0 and tar_x < self.num_position_x and tar_y >= 0 and tar_y < self.num_position_y:
            assert self.maze.cell_at(x ,y).walls[direction] == self.maze.cell_at(tar_x , tar_y).walls[reverse_direction]
        return self.maze.cell_at(x ,y).walls[direction] if self.use_wall else (tar_x < 0 or tar_x >= self.num_position_x or tar_y < 0 or tar_y >= self.num_position_y)

    def get_reachabel_loc(self, x, y):
        reachabel = [(x-1,y), (x,y-1), (x,y+1), (x+1,y)]
        reachabel = set(filter(lambda t: not self.block_by_wall(x, y, t[0], t[1]), reachabel))
        return reachabel

    def encode_constraints(self):
        all_cons = [self.encode_init_state, self.encode_goal_state, self.encode_action_schema, self.encode_state_schema]  #
        all_f = []
        for func in all_cons:
            partial_fomula = func()
            all_f.append(partial_fomula)
        final_formula = And(all_f)
        return final_formula

    def encode_init_state(self):
        all_f = []
        for n_robot in range(self.num_robot):
            for i in range(self.num_position_x):
                for j in range(self.num_position_y):
                    if i == self.init_robot_loc[0] and j == self.init_robot_loc[1]:
                        all_f.append(self.at[n_robot, i, j, self.init_time_step])
                    else:
                        all_f.append(Not(self.at[n_robot, i, j, self.init_time_step]))
        return And(all_f)

    def encode_goal_state(self):
        all_possible_goal = []
        for n_robot in range(self.num_robot):
            loc_x, loc_y = self.goal_robot_loc
            for time in self.goal_time_steps:
                all_possible_goal.append(self.at[n_robot, loc_x, loc_y, time])
        return Or(all_possible_goal)

    def encode_action_schema(self):
        all_f = []
        for n_robot in range(self.num_robot):
            for time_step_action in range(self.max_time_step):
                for i in range(self.num_position_x):
                    for j in range(self.num_position_y):
                        reachable = self.get_reachabel_loc(i, j)
                        for tar_i, tar_j in reachable:
                            # print(n_robot, i, j, tar_i, tar_j, time_step_action)
                            action = self.move[n_robot, i, j, tar_i, tar_j, time_step_action]
                            pre, add, dele = self.get_schema(self.move, (n_robot, i, j, tar_i, tar_j, time_step_action))

                            #### precondition need to be satisfied to take an action ###
                            all_f.append(Implies(action, pre))
                            #### an action changes the state on next time step ###
                            all_f.append(Implies(action, add))
                            #### an action changes the state on next time step ###
                            all_f.append(Implies(action, Not(dele)))
        return And(all_f)

    def encode_state_schema(self):
        all_f = []
        for n_robot in range(self.num_robot):
            for time_step_action in range(self.max_time_step):
                for i in range(self.num_position_x):
                    for j in range(self.num_position_y):
                        reachable = self.get_reachabel_loc(i, j)
                        possible_out_move = []
                        possible_in_move = []
                        for tar_i, tar_j in reachable:
                            # print(n_robot, i, j, tar_i, tar_j, time_step_action)
                            possible_out_move.append(self.move[n_robot, i, j, tar_i, tar_j, time_step_action])
                            possible_in_move.append(self.move[n_robot, tar_i, tar_j, i, j, time_step_action])
                        all_f.append(Implies(And(Not(self.at[n_robot, i, j, time_step_action]), self.at[n_robot, i, j, time_step_action+1]),
                                                 ExactlyOne(possible_in_move)))
                        all_f.append(Implies(And(self.at[n_robot, i, j, time_step_action], Not(self.at[n_robot, i, j, time_step_action+1])),
                                                 ExactlyOne(possible_out_move)))
        return And(all_f)

    def parse_solution(self):
        position = [self.at.used_mapping[k] for k in self.at.get_used_keys() if self.solver.get_value(k).is_true()]
        sol = []
        for n_robot in range(self.num_robot):
            n_robot_position = list(filter(lambda x: x[1][0] == n_robot, position))
            sorted_position = list(map(lambda t: t[1][1:], sorted(n_robot_position, key=lambda x: x[1][3])))
            sol.append(sorted_position)
            print(sorted_position)
            print(len(sorted_position))
        self.maze.write_svg("maze_solution.svg", solution=sol)

    def solve_for_one(self):
        while self.solver.solve():
            partial_model = [EqualsOrIff(k, self.solver.get_value(k)) for k in self.move.get_used_keys().union(self.at.get_used_keys())]
            self.parse_solution()
            self.solver.add_assertion(Not(And(partial_model)))
            return tuple(partial_model)
        return None


if __name__ == "__main__":
    nx, ny = 5, 5
    max_time = 50
    mz = Maze(0, nx, ny, 0, 0)
    mz.make_maze()
    mz.write_svg('maze.svg')

    prob = Robot_Plan(num_robot=1, maze=mz, max_time=max_time, use_wall=True)
    solution_set = set()
    while True:
        solution = prob.solve_for_one()
        # exit(0)
        if solution is not None:
            if solution in solution_set:
                raise Exception("Duplicated Solution!")
            solution_set.add(solution)
        else:
            break







