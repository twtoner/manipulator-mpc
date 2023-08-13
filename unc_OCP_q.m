clear;clc;close all
importKuka

dt = 0.01;
m = 100;
n = 7;
% 
% pg = [0.75; 0; 0.4];
% Rg = eul2rotm([0, pi/2, 0]);
% Tg = [Rg, pg; 0, 0, 0, 1];
% q0 = [0.3559   -0.5249    0.0939   -2.0591    1.6915   -0.4401   -0.0814]';

q0 = robot.randomConfiguration;
qg = robot.randomConfiguration;

A = eye(n); B = dt * eye(n);
Q = 0.05 * eye(n);  
Ru = 1e-3 * eye(n);  
P = Q;
[S, M, Qbar, Rbar, ~] = uncMPC(m, A, B, Q, Ru, P);

H = S' * Qbar * S + Rbar;
p = S' * Qbar * M;

tic
U = -inv(H) * p * (q0 - qg);
toc
u = reshape(U, n, m);
q = q0 + dt * cumsum(u,2);
err = q - qg;

figure
subplot(3,1,1);
plot(err', '*--'); 
title('joint error')

subplot(3,1,2)
plot(q', '*--');
title('joint traj.')

subplot(3,1,3)
plot(u', '*--');
title('control input')


