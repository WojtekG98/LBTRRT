from ompl import base as ob
from ompl import geometric as og
import matplotlib.pyplot as plt
from math import sqrt
from math import pi
import sys
import Astar
import RRT
import random
import ciesnina
import labirynt
import prostokat as prostokatfile

cies = 0
prostokat = 0
lab = 1
inna = 0

N = 100.0
dgoal = 50


class IterationPTC(object):
    def __init__(self, maxIter):
        self.iter = 0
        self.maxIter = maxIter

    def __call__(self):
        self.iter += 1
        return self.iter >= self.maxIter

def plan(space, planner, maxIter, start, goal):
    ss = og.SimpleSetup(space)
    if cies == 1:
        ss.setStateValidityChecker(ob.StateValidityCheckerFn(ciesnina.isStateValid))
    if prostokat == 1:
        ss.setStateValidityChecker(ob.StateValidityCheckerFn(prostokatfile.isStateValid))
    if lab == 1:
        ss.setStateValidityChecker(ob.StateValidityCheckerFn(labirynt.isStateValid))

    ss.setStartAndGoalStates(start, goal)
    if planner == 'RRT':
        ss.setPlanner(RRT.RRT(ss.getSpaceInformation()))
    elif planner == 'rrt':
        ss.setPlanner(og.RRT(ss.getSpaceInformation()))
    elif planner.lower() == 'lbtrrt':
        ss.setPlanner(og.LBTRRT(ss.getSpaceInformation()))
    elif planner.lower() == 'astar':
        ss.setPlanner(Astar.Astar(ss.getSpaceInformation()))
    elif planner.lower() == "rrtconnect":
        ss.setPlanner(og.RRTConnect(ss.getSpaceInformation()))
    elif planner.lower() == "est":
        ss.setPlanner(og.EST(ss.getSpaceInformation()))
    elif planner.lower() == "rrtstar":
        ss.setPlanner(og.RRTstar(ss.getSpaceInformation()))
    elif planner.lower() == "informedrrtstar":
        ss.setPlanner(og.InformedRRTstar(ss.getSpaceInformation()))
    elif planner.lower() == "sst":
        print("SST")
        ss.setPlanner(og.SST(ss.getSpaceInformation()))
    else:
        print('Bad planner')
    print("\n")
    print(planner, ":")
    #OptObj = ob.PathLengthOptimizationObjective(ss.getSpaceInformation())
    #OptObj.setCostThreshold(c_best)
    #ss.setOptimizationObjective(OptObj)
    #print("OptObj", ss.getOptimizationObjective().getDescription())
    iptc = IterationPTC(maxIter)
    solved = ss.solve(ob.PlannerTerminationConditionFn(iptc))
    print("Time:", ss.getLastPlanComputationTime())
    if solved:
        PD = ob.PlannerData(ss.getSpaceInformation())
        ss.getPlannerData(PD)
        print("numVertices:", PD.numVertices())
        print("numIterations:", iptc.iter)
        ss.simplifySolution()
        path = ss.getSolutionPath()
        print("Info:    Path length:", path.length())
        # print(path.printAsMatrix())
        path.interpolate(1000)
        if planner.lower() == 'astar':
            return path.length()
        else:
            return path.printAsMatrix()

    else:
        print("No solution found.")
        return None


def print_path_txt(path):
    plt.axis([0, N, 0, N])
    matrix = []
    for line in path.split("\n"):
        tmp = []
        for item in line.split():
            tmp.append(float(item))
        if len(tmp) is not 0:
            matrix.append(list(tmp))
    for item in matrix:
        item[2] = item[2] * 180 / pi
    return matrix


def plot_path(path, style, label, l, h):
    plt.axis([l, h, l, h])
    matrix = []
    for line in path.split("\n"):
        tmp = []
        for item in line.split():
            tmp.append(float(item))
        if len(tmp) is not 0:
            matrix.append(list(tmp))
    x = []
    y = []
    for item in matrix:
        x.append(item[0])
        y.append(item[1])
    plt.plot(x, y, style, label=label)

def paintobs():
    if cies == 1:
        ciesnina.paint_obs(0, N)
    if prostokat == 1:
        prostokatfile.paint_obs(0, N)
    if lab == 1:
        labirynt.paint_obs(0, N)

def plot_path_to_png(path, style, LB, HB, fignum, legend, pathtofile, s, g):
    plt.figure(fignum)
    if path:
        plot_path(path, style, legend, LB, HB)
        plt.plot(s[0], s[1], 'k*')
        plt.plot(g[0], g[1], 'y*')
        paintobs()
        plt.legend()
        plt.gca().set_aspect('equal', adjustable='box')
        plt.savefig(pathtofile)

def set_start_and_goal(start, goal):
    if cies == 1 or prostokat == 1:
        start[0], start[1] = (N-dgoal)/2, N/2
        goal[0], goal[1] = (N+dgoal)/2, N/2
    if lab == 1:
        start[0], start[1] = 10, 5
        goal[0], goal[1] =  80, 75
    if inna == 1:
        start[0], start[1] = random.randint(0, N), random.randint(0, N)
        goal[0], goal[1] = random.randint(0, N), random.randint(0, N)
        if cies == 1:
            while not ciesnina.isStateValid2(start):
                start[0], start[1] = random.randint(0, N), random.randint(0, N / 2)
            while not ciesnina.isStateValid2(goal):
                goal[0], goal[1] = random.randint(0, N), random.randint(N / 2, N)
        if prostokat == 1:
            while not prostokatfile.isStateValid2(start):
                start[0], start[1] = random.randint(0, N), random.randint(0, N / 2)
            while not prostokatfile.isStateValid2(goal):
                goal[0], goal[1] = random.randint(0, N), random.randint(N / 2, N)
        if lab == 1:
            while not labirynt.isStateValid2(start):
                start[0], start[1] = random.randint(0, N), random.randint(0, N)
            while not labirynt.isStateValid2(goal):
                goal[0], goal[1] = random.randint(0, N), random.randint(0, N)

def badanie(nr, maxIter, space, start, goal):
    nazwa_pliku = "badania/" + str(int(nr)) + "I" + str(int(maxIter)) + ".txt"
    f = open(nazwa_pliku, 'w')
    #sys.stdout = f

    rrtstar_path = plan(space, 'rrtstar', maxIter, start, goal)
    plot_path_to_png(rrtstar_path, 'g-', 0, N, 1, 'RRT*', 'figures/path_RRTStar.png', start, goal)

    sst_path = plan(space, 'sst', maxIter, start, goal)
    plot_path_to_png(sst_path, 'm-', 0, N, 2, 'SST', 'figures/path_SST.png', start, goal)

    # rrtconnect_path = plan(space, 'rrtconnect', maxIter, start, goal)
    # plot_path_to_png(rrtconnect_path, 'r-', 0, N, 3, ('RRTConnect', 'start', 'goal'), 'figures/path_RRTConnect.png')

    informedrrtstar_path = plan(space, 'informedrrtstar', maxIter, start, goal)
    plot_path_to_png(informedrrtstar_path, 'b-', 0, N, 3, 'InformedRRT*', 'figures/path_InformedRRTStar.png', start, goal)

    lbtrrt_path = plan(space, 'lbtrrt', maxIter, start, goal)
    plot_path_to_png(lbtrrt_path, 'r-', 0, N, 4, 'LBTRRT', 'figures/LBTRRT_path.png', start, goal)

    f.write("end")
    f.close()

    plt.figure(5)
    paintobs()
    # plot_path(rrtconnect_path, 'r-', 0, N)
    plot_path(rrtstar_path, 'b-', 'RRT*', 0, N)
    plot_path(sst_path, 'g-', 'SST', 0, N)
    plot_path(informedrrtstar_path, 'r-', 'InformedRRT*', 0, N)
    plot_path(lbtrrt_path, 'm-', 'LBTRRT', 0, N)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.plot(start[0], start[1], 'k*')
    plt.plot(goal[0], goal[1], 'y*')
    plt.legend()
    plt.savefig('figures/paths.png')


if __name__ == '__main__':
    space = ob.ReedsSheppStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0)
    bounds.setHigh(N)
    space.setBounds(bounds)
    # Set our robot's starting and goal states to be random
    start, goal = ob.State(space), ob.State(space)
    set_start_and_goal(start, goal)
    print("start: ", start[0], start[1])
    print("goal: ", goal[0], goal[1])
    badanie(1, 1000, space, start, goal)