clear;clc;
close all
importKuka

% Tracking joint config with formulation 1.

ik = inverseKinematics('RigidBodyTree', robot);

dt = 0.01;
Nmax = 500;
t = 0 : dt : (Nmax-1)*dt;
n = 7;
N = Nmax;

q0 = robot.randomConfiguration;

qg = robot.randomConfiguration;

A = eye(n); 
B = dt * eye(n);
Ru = eye(n);  
Q = 1e10 * eye(n);  
P = Q;
[S, M, Qbar, Rbar, ~] = uncMPC(N, A, B, Q, Ru, P);
H = 2 * S' * Qbar * S + Rbar;
f0 = 2 * S' * Qbar * M;
f = f0 * (q0 - qg);

% Constraints
u_UB = (pi/180) * [98 98 100 130 140 180 180]';
u_LB = -u_UB;
U_UB = repmat(u_UB, [N, 1]);
U_LB = repmat(u_LB, [N, 1]);
q_UB = [2.9671 2.0944 2.9671 2.0944 2.9671 2.0944 3.0543]';
q_LB = -q_UB;
Q_UB = repmat(q_UB, [N, 1]);
Q_LB = repmat(q_LB, [N, 1]);
G = [S; -S; eye(size(S)); -eye(size(S))];
W = [Q_UB; -Q_LB; U_UB; -U_LB];
T = [-M; M; zeros(size(M)); zeros(size(M))];
Wtil = W + T * q0;

U0 = zeros(N*n, 1);
Ustar = quadprog(H, f, G, Wtil, [], [], U_LB, U_UB, U0);

Q = S*Ustar + M*q0;
u = reshape(Ustar, n, []);
q = reshape(Q, n, []);
%%
err = q - qg;

fig = figure;
% fig.Position = [1000 1200 500 1000];  % office PC
fig.Position = [3825 -300 1920 1080];     % home PC side monitor

subplot(3,1,1);
plot(t, err', '*--')
grid on
title('joint error')

subplot(3,1,2)
hold on
plot(t, q', '*--');
yline(q_UB, 'r--');
yline(q_LB, 'b--');
hold off
grid on
title('joint traj.')

subplot(3,1,3)
hold on
plot(t, u', '*--');
yline(u_UB, 'r--')
yline(u_LB, 'b--')
hold off
grid on
title('control input')

disp(['final error: ', num2str(err(:,end)')]);
disp(['error norm: ', num2str(norm(err(:,end)))]);

%%
% visualizeRobotTraj(robot, q, t, [0; 0; 0], [0; 0; 0], 2)
