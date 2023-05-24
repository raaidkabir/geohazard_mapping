clc;
clear;
close all;
load("perlin.mat")
A = 1;
B = 2;
C = 0.01;
D = 1;
numIter = 500;
KrigingTraverseMaze(I, A, B, C, D, numIter)