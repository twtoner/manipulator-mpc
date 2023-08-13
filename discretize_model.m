clear;clc

n = 7;

Ac = zeros(7,7);
Bc = eye(7);
C = eye(7);
D = 0;
Ts = 0.01;
sys = c2d(ss(Ac, Bc, C, D), Ts)
