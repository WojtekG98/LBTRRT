from ompl import base as ob
from ompl import geometric as og
import matplotlib.pyplot as plt
import sys
import labirynt
from Pathfun import plot_path, plot_path_to_png, paintobs, plot_legend
from Ustawienia import N, lab, dgoal, inna, L, des_time


def plan(space, planner, time, start, goal):
    ss = og.SimpleSetup(space)
    if lab == 1:
        ss.setStateValidityChecker(ob.StateValidityCheckerFn(labirynt.isStateValid))

    ss.setStartAndGoalStates(start, goal)
    if planner == 'rrt':
        ss.setPlanner(og.RRT(ss.getSpaceInformation()))
    elif planner.lower() == 'lbtrrt':
        ss.setPlanner(og.LBTRRT(ss.getSpaceInformation()))
    elif planner.lower() == "rrtconnect":
        ss.setPlanner(og.RRTConnect(ss.getSpaceInformation()))
    elif planner.lower() == "est":
        ss.setPlanner(og.EST(ss.getSpaceInformation()))
    elif planner.lower() == "rrtstar":
        ss.setPlanner(og.RRTstar(ss.getSpaceInformation()))
    elif planner.lower() == "informedrrtstar":
        ss.setPlanner(og.InformedRRTstar(ss.getSpaceInformation()))
    elif planner.lower() == "sst":
        ss.setPlanner(og.SST(ss.getSpaceInformation()))
    else:
        print('Bad planner')
        return 0, None
    OptObj = ob.PathLengthOptimizationObjective(ss.getSpaceInformation())
    # OptObj.setCostThreshold(c_best)
    ss.setOptimizationObjective(OptObj)
    solved = ss.solve(time)
    if solved:
        ss.simplifySolution()
        path = ss.getSolutionPath()
        if path.getState(path.getStateCount()).getX() == goal[0] and \
                path.getState(path.getStateCount()).getY() == goal[1]:
            found = 1
        else:
            found = 0

        path.interpolate(1000)
        return found, path.printAsMatrix(), path.length()
    else:
        print("No solution found.")
        return 0, None


def set_start_and_goal(start, goal):
    if lab == 1:
        start[0], start[1] = 10, 5
        goal[0], goal[1] = 80, 75


def badanie(time, space, start, goal):
    rrtstar_path = plan(space, 'rrtstar', time, start, goal)
    plot_path_to_png(rrtstar_path[1], 'g-', 0, N, 1, 'RRT*',
                     'figures/path_RRTStar.png', start, goal)

    sst_path = plan(space, 'sst', time, start, goal)
    plot_path_to_png(sst_path[1], 'm-', 0, N, 2, 'SST',
                     'figures/path_SST.png', start, goal)

    informedrrtstar_path = plan(space, 'informedrrtstar', time, start, goal)
    plot_path_to_png(informedrrtstar_path[1], 'b-', 0, N, 3, 'InformedRRT*',
                     'figures/path_InformedRRTStar.png', start, goal)

    lbtrrt_path = plan(space, 'lbtrrt', time, start, goal)
    plot_path_to_png(lbtrrt_path[1], 'r-', 0, N, 4, 'LBTRRT',
                     'figures/path_LBTRRT.png', start, goal)

    plt.figure(5)
    paintobs(1)
    # plot_path(rrtconnect_path, 'r-', 0, N)
    plot_path(rrtstar_path[1], 'b-', 0, N)
    plot_path(sst_path[1], 'g-', 0, N)
    plot_path(informedrrtstar_path[1], 'r-', 0, N)
    plot_path(lbtrrt_path[1], 'm-', 0, N)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.plot(start[0], start[1], 'k*')
    plt.plot(goal[0], goal[1], 'y*')
    plot_legend(('RRT*', 'SST', 'InformedRRT*', 'LBTRRT',))
    plt.savefig('figures/paths.png')
    return (rrtstar_path[0], rrtstar_path[2]), (sst_path[0], sst_path[2]),\
           (informedrrtstar_path[0], informedrrtstar_path[2]), (lbtrrt_path[0], lbtrrt_path[2])


if __name__ == '__main__':
    s = ob.ReedsSheppStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0)
    bounds.setHigh(N)
    s.setBounds(bounds)
    # Set our robot's starting and goal states to be random
    start_, goal_ = ob.State(s), ob.State(s)
    set_start_and_goal(start_, goal_)
    print("start: ", start_[0], start_[1])
    print("goal: ", goal_[0], goal_[1])
    wynik = list([(0, 0), (0, 0), (0, 0), (0, 0)])
    wynik[:] = [list(x) for x in wynik]
    nazwa_pliku = "badania/" + str(L) + "_" + str(des_time) + "s.txt"
    f = open(nazwa_pliku, 'w')
    sys.stdout = f
    print("time:", des_time)
    for n in range(0, L):
        tmp = badanie(des_time, s, start_, goal_)
        wynik_rrtstar = tmp[0]
        wynik_sst = tmp[1]
        wynik_informedrrtstar = tmp[2]
        wynik_lbtrrt = tmp[3]
        wynik[0][0] = wynik[0][0] + wynik_rrtstar[0]
        wynik[0][1] = wynik[0][1] + wynik_rrtstar[1]
        wynik[1][0] = wynik[0][0] + wynik_sst[0]
        wynik[1][1] = wynik[0][1] + wynik_sst[1]
        wynik[2][0] = wynik[0][0] + wynik_informedrrtstar[0]
        wynik[2][1] = wynik[0][1] + wynik_informedrrtstar[1]
        wynik[3][0] = wynik[0][0] + wynik_lbtrrt[0]
        wynik[3][1] = wynik[0][1] + wynik_lbtrrt[1]
    times = [x[0] / L for x in wynik]
    length = [x[1] / L for x in wynik]
    names = ('rrtstar', 'sst', 'informedrrtstar', 'lbtrrt')
    print(list(names))
    print(times)
    print(length)
    f.close()
    sys.stdout = sys.__stdout__
    print(list(names))
    print(times)
    print(length)
