clc; clear; close all;
addpath("../Astar/","../variogramfit/","../variogram/", "../kriging/", "../FMINSEARCHBND/FMINSEARCHBND/","../perlin_matlab/")
load("perlin.mat")
A = 1;
B = 10;
C = 0.01;
D = 1;
E = 0.03;
numIter = 500;
KrigingTraverseMaze_multirobot(I, A, B, C, D, E, numIter, 3)