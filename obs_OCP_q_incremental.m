clear;clc;
close all
importKuka

% Tracking joint config with formulation 1.

ik = inverseKinematics('RigidBodyTree', robot);

dt = 0.05;
Nmax = 50;
t = 0 : dt : (Nmax-1)*dt;
n = 7;
N = Nmax;
Ni = 5;    % increment length

[geom_xyz, q0, no, ~, ~, Tg, ~, ~, ~, ~, ~, ~, ~, ~, ~] = setupProblem();
q0 = q0';
r = 0.09 * ones(7,1);
% downsample
geom_xyz_ds = pcdownsample( pointCloud(geom_xyz'), 'random', 0.2).Location';

% q0 = robot.randomConfiguration;
% qg = robot.randomConfiguration;

qg = [0.6739    1.4631    1.5522    1.3407   -0.1543    0.6711    0.0684]';


A = eye(n); 
B = dt * eye(n);
Ru = eye(n);  
Q = 1e11 * eye(n);  
% Q = 1e14 * eye(n);  
P = Q;
[S, M, Qbar, Rbar, ~] = uncMPC(Ni, A, B, Q, Ru, P);
H = 2 * S' * Qbar * S + Rbar;
f0 = 2 * S' * Qbar * M;
f = f0 * (q0 - qg);

% Constraints
u_UB = (pi/180) * [98 98 100 130 140 180 180]';
u_LB = -u_UB;
U_UB = repmat(u_UB, [Ni, 1]);
U_LB = repmat(u_LB, [Ni, 1]);
q_UB = [2.9671 2.0944 2.9671 2.0944 2.9671 2.0944 3.0543]';
q_LB = -q_UB;
Q_UB = repmat(q_UB, [Ni, 1]);
Q_LB = repmat(q_LB, [Ni, 1]);
G = [S; -S; eye(size(S)); -eye(size(S))];
W = [Q_UB; -Q_LB; U_UB; -U_LB];
T = [-M; M; zeros(size(M)); zeros(size(M))];
Wtil = W + T * q0;

% U_UB = RTF * U_UB;
% U_LB = RTF * U_LB;
% U_UB = [];
% U_LB = [];

ops = optimoptions('fmincon', 'Algorithm', 'sqp', ...
        'SpecifyObjectiveGradient', true, 'UseParallel', true, ...
        'SpecifyConstraintGradient', false, ...
        'Display', 'iter', 'PlotFcn', {'optimplotfvalconstr', 'optimplotx', 'optimplotfval'}, ...
        'MaxFunctionEvaluations', 1e6);
U0 = zeros(Ni*n, 1);
q = q0;
u = [0*q0];
k = 1;
for i = 1:floor(N/Ni)-1
    [Ustar, fval, exitflag] = fmincon(@(U) costFcn_q(U, S, M, Qbar, Rbar, q0, qg), ...
                U0, G, Wtil, [], [], U_LB, U_UB, ...
                @(U) collisionFcn_U(U, geom_xyz_ds, r, S, M, q0), ops);
    Q = S * Ustar + M * q0;
    q = [q, reshape(Q, n, [])];
    u = [u, reshape(Ustar, n, [])];
    q0 = q(:,end);
    
    disp(['%%%%%%%%%%%%%%%%%%%%%  iteration: ', num2str(100 * (i / floor(N/Ni)))])

end

% Q = S*Ustar + M*q0;
% u = reshape(Ustar, n, []);
% q = reshape(Q, n, []);
%%
err = q - qg;

fig = figure;
fig.Position = [1000 1200 500 1000];  % office PC
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

%% compute mindist
for i = 1:N
    D = squeeze(vecnorm(getDIJ_proj(q(:,i), geom_xyz_ds)));
    margin(i) = min(D-r, [], 'all');    
end

figure
plot(t, margin)
%%
figure
pg = Tg(1:3, 4);
visualizeRobotTraj(robot, q, t, pg, geom_xyz_ds, 1)
