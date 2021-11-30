from math import pi

from matplotlib import pyplot as plt

import labirynt
from Ustawienia import N

def plot_legend(label):
    #plt.gca().set_label(label)
    plt.legend(label)


def plot_path(path, style, l, h):
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
    plt.plot(x, y, style)


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


def plot_path_to_png(path, style, LB, HB, fignum, legend, pathtofile, s, g):
    plt.figure(fignum)
    if path:
        plot_path(path, style, LB, HB)
        plt.plot(s[0], s[1], 'k*')
        plt.plot(g[0], g[1], 'y*')
        paintobs(1)
        plot_legend(list((legend, 'start', 'goal')))
        plt.gca().set_aspect('equal', adjustable='box')
        plt.savefig(pathtofile)


def paintobs(obs):
    if obs == 1:
        labirynt.paint_obs(0, N)