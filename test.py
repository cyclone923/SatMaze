from argparse import ArgumentParser
from exps.simply_robot import Robot_Plan
from tool.maze import Maze
import time

def parse_args():
    parser = ArgumentParser()

    # crucial arguments
    parser.add_argument('-s', '--seed', default=0, type=int,
                        help='random seed for torch and gym')

    args = parser.parse_args()
    return args

def main():
    args = parse_args()

    nx, ny = 10, 10
    max_time = 90
    mz = Maze(args.seed, nx, ny, 0, 0)
    mz.make_maze()
    mz.write_svg('maze.svg')

    prob = Robot_Plan(num_robot=1, maze=mz, max_time=max_time, use_wall=True)
    solution_set = set()
    while True:
        time0 = time.time()
        solution = prob.solve_for_one()
        time1 = time.time()
        print(time1-time0)
        exit(0)
        if solution is not None:
            if solution in solution_set:
                raise Exception("Duplicated Solution!")
            solution_set.add(solution)
        else:
            break

if __name__ == '__main__':
    main()


