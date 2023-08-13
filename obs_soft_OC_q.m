clear;clc;
close all
importKuka

% Tracking joint config with formulation 1. Same as obs_OC_q except with
% cost function that penalizes obstacle proximity. 

ik = inverseKinematics('RigidBodyTree', robot);

dt = 0.1;
Nmax = 30;
t = 0 : dt : (Nmax-1)*dt;
n = 7;
N = Nmax;

[geom_xyz, q0, no, ~, ~, Tg, ~, ~, ~, ~, ~, ~, ~, ~, ~] = setupProblem();
q0 = q0';
r = 0.12 * ones(7,1);
% downsample
geom_xyz_ds = pcdownsample( pointCloud(geom_xyz'), 'gridAverage', 0.1).Location';
no_ds = size(geom_xyz_ds, 2);
% q0 = robot.randomConfiguration;
% qg = robot.randomConfiguration;

qg = [0.6739    1.4631    1.5522    1.3407   -0.1543    0.6711    0.0684]';

q0 = qg;

A = eye(n); 
B = dt * eye(n);
Ru = eye(n);  
% Q = 1e11 * eye(n);  
% Q = 1e14 * eye(n);  
% P = Q;
Q = 1e4 * eye(n);
P = 1e12 * Q;
[S, M, Qbar, Rbar, ~] = uncMPC(N, A, B, Q, Ru, P);
H = 2 * S' * Qbar * S + Rbar;
f0 = 2 * S' * Qbar * M;
f = f0 * (q0 - qg);

% Constraints
u_UB = (pi/180) * [98 98 100 130 140 180 180]';     u_UB = 1 * u_UB;
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

% u_avg = (qg - q0) / (Nmax*dt);
% u_avg = min(u_UB, u_avg*1.01);
% u_avg = max(u_LB, u_avg*0.99);
% U0 = repmat(u_avg, N, 1);

% U_UB = RTF * U_UB;
% U_LB = RTF * U_LB;
% U_UB = [];
% U_LB = [];
%%
ops = optimoptions('fmincon', 'Algorithm', 'interior-point', ...
        'SpecifyObjectiveGradient', true, 'UseParallel', false, ...
        'SpecifyConstraintGradient', true, ...
        'Display', 'iter', 'PlotFcn', {'optimplotfvalconstr', 'optimplotx', 'optimplotfval'}, ...
        'MaxIterations', 300);
    
U0 = zeros(N*n, 1);
tic
[c, ceq, gradC, gradCeq] = collisionFcn_U_mex(U0, geom_xyz_ds, r, S, M, q0);
toc

tic
[J, dJdU] = costFcn_q(U0, S, M, Qbar, Rbar, q0, qg);
toc


tic
[Ustar, fval, exitflag] = fmincon(@(U) costFcn_q_soft(U, geom_xyz_ds, S, M, Qbar, Rbar, q0, qg), ...
                U0, G, Wtil, [], [], U_LB, U_UB, ...
                @(U) collisionFcn_U_mex(U, geom_xyz_ds, r, S, M, q0), ops);
% [Ustar, fval, exitflag] = fmincon(@(U) costFcn_q_soft(U, geom_xyz_ds, S, M, Qbar, Rbar, q0, qg), ...
%                 U0, G, Wtil, [], [], U_LB, U_UB, ...
%                 [], ops);
toc

Q = S*Ustar + M*q0;
u = reshape(Ustar, n, []);
q = reshape(Q, n, []);
%%
err = q - qg;

fig = figure;
% fig.Position = [1000 1200 500 1000];  % office PC
% fig.Position = [3825 -300 1920 1080];     % home PC side monitor

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
figure
pg = Tg(1:3, 4);
visualizeRobotTraj(robot, q, t, pg, geom_xyz, 1.0)

%% 
% Check gradient costFcn_q
% U = randn(N*n, 1);
% 
% [~, dJdU] = costFcn_q(U, S, M, Qbar, Rbar, q0, qg);
% 
% dJdU2 = zeros(size(dJdU));
% 
% dU = 1e-4;
% for i = 1:N*n
%     dU_ = zeros(N*n,1);
%     dU_(i) = dU;
%     U_more = U + dU_;
%     U_less = U - dU_;
%     [J_more, ~] = costFcn_q(U_more, S, M, Qbar, Rbar, q0, qg);
%     [J_less, ~] = costFcn_q(U_less, S, M, Qbar, Rbar, q0, qg);
%     dJdU2(i) = (J_more - J_less) / (2*dU);
% end
% 
% percent_err = 100 * (dJdU - dJdU2) ./ dJdU

%% Check gradient costFcn_q_soft
% O = geom_xyz_ds;
% U = Ustar;
% 
% [J, dJdU] = costFcn_q_soft(U, O, S, M, Qbar, Rbar, q0, qg);
% 
% dJdU_num = zeros(size(dJdU));
% dU = 1e-6;
% for i = 1:n*N
%     dU_ = zeros(n*N, 1);
%     dU_(i) = dU;
%     U_more = U + dU_;
%     U_less = U - dU_;
%     [J_more, ~] = costFcn_q_soft(U_more, O, S, M, Qbar, Rbar, q0, qg);
%     [J_less, ~] = costFcn_q_soft(U_less, O, S, M, Qbar, Rbar, q0, qg);
%     dJdU_num(i) = (J_more - J_less)/(2*dU);
% end
% 
% percent_err = 100 * (dJdU - dJdU_num) ./ dJdU


