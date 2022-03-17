clear; clc;
format compact;
omega = 1;
omega2 = 0.5;
H = 100;
acc=0;

A = [0 1 0 0 0 0; 
0 0 0 0 0 0; 
0 0 0 omega 0 0; 
0 0 -omega 0 0 0;
0 0 0 0 0 omega2;
0 0 0 0 -omega2 0];