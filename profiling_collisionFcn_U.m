clear;clc;
close all

dt = 0.1;
Nmax = 50;
t = 0 : dt : (Nmax-1)*dt;
n = 7;
N = Nmax;

[geom_xyz, q0, no, ~, ~, Tg, ~, ~, ~, ~, ~, ~, ~, ~, ~] = setupProblem();

q0 = q0';
r = 0.09 * ones(7,1);
% downsample
geom_xyz_ds = pcdownsample( pointCloud(geom_xyz'), 'random', 0.01).Location';
no_ds = size(geom_xyz_ds, 2);

qg = [0.6739    1.4631    1.5522    1.3407   -0.1543    0.6711    0.0684]';

A = eye(n); 
B = dt * eye(n);
Ru = eye(n);  
Q = 1e14 * eye(n);  
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

% [c, ceq, gradC, gradCeq] = collisionFcn_U(U0, geom_xyz_ds, r, S, M, q0);

tic
% Precompute h
%     tform = getTransforms_iiwa7_mex(q0);
%     p = squeeze(tform(1:3, 4, :));
%     p17 = p(:, 1:7);
%     pjj = diff(p,1,2);
%     h = zeros(no_ds, 7);
%     for i = 1:no_ds
%         h(i,:) = dot(geom_xyz_ds(:,i) - p17, pjj) ./ dot(pjj, pjj);
%     end
% h = get_h(q0, geom_xyz_ds);
% [c, ceq, gradC, gradCeq] = collisionFcn_U_h(U0, geom_xyz_ds, r, S, M, q0, h);
% [c, ceq, gradC, gradCeq] = collisionFcn_U_h_mex(U0, geom_xyz_ds, r, S, M, q0, h);
% [c, ceq, gradC, gradCeq] = collisionFcn_U(U0, geom_xyz_ds, r, S, M, q0);
[c, ceq, gradC, gradCeq] = collisionFcn_U_mex(U0, geom_xyz_ds, r, S, M, q0);

toc

