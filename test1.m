global K
K=476;
global TERMINAL_STATE_INDEX
TERMINAL_STATE_INDEX=90
clc
clear all
global K
K=476;
global TERMINAL_STATE_INDEX
TERMINAL_STATE_INDEX=90
P_exp=load('example_P.mat');
P=P_exp.P;
G1=load("example_G.mat");
G=G1.G;

% TODO: Question d)
[ J_opt_lp, u_opt_ind_lp ] = LinearProgramming(P, G);