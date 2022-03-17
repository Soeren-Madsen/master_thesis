clear; clc;
format compact;
omega = 1;
omega2 = 0.5;
acc = 0;
H = 100;
%One sine wave
%A = [0 1 0 0; 0 0 0 0; 0 0 0 omega; 0 0 -omega 0];

%Two sine waves
A = [0 1 0 0 0 0; 
0 0 0 0 0 0; 
0 0 0 omega 0 0; 
0 0 -omega 0 0 0;
0 0 0 0 0 omega2;
0 0 0 0 -omega2 0];

%Three sine waves
% A = [0 1 0 0 0 0 0 0; 
% 0 0 0 0 0 0 0 0; 
% 0 0 0 omega 0 0 0 0; 
% 0 0 -omega 0 0 0 0 0;
% 0 0 0 0 0 omega2 0 0;
% 0 0 0 0 -omega2 0 0 0;
% 0 0 0 0 0 0 0 omega3;
% 0 0 0 0 0 0 -omega3 0];