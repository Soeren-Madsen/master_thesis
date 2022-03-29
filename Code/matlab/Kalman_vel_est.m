clear; clc;
format compact;
%A = [0 1 0 0; 0 0 0 0; 0 0 0 1; 0 0 0 0];
%B = [0; 1; 0; 0];
%C = [1 0 -1 0; 1 0 0 0];

A = [0 1 0 0 0; 0 0 0 0 0; 0 0 0 1 0; 0 0 0 0 1; 0 0 0 0 0];
B = [0; 1; 0; 0; 0];
C = [1 0 -1 0 0; 1 0 0 0 0];
D = 0;

sys = ss(A, B, C, D);

Ob = obsv(sys);

unob = length(A)-rank(Ob)
dt = 0.1
%
%Q = [1000 0 0 0 0; 0 1000 0 0 0; 0 0 1000 0 0; 0 0 0 3000 0; 0 0 0 0 9000];
%Q = eye(4);
Q = [dt^2 dt 0 0 0; dt 1 0 0 0; 0 0 dt^4/4 dt^3/2 dt^2/2; 0 0 dt^3/2 dt^2 dt; 0 0 dt^2/2 dt 1]
%Q = Q*100
R = [1 0; 0 1];

%x0 = [10 0 0 1];
x0 = [10 0 0 1 0];

%Discrete time

sys_d = c2d(sys,dt)
