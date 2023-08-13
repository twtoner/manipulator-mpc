clear;clc;
close all
importKuka

% Tracking q with formulation 1 and obstacles.

ik = inverseKinematics('RigidBodyTree', robot);

dt = 0.1;
Nmax = 100;
t = 0 : dt : (Nmax-1)*dt;
n = 7;
N = 10;

[geom_xyz, q0, no, ~, ~, Tg, ~, ~, ~, ~, ~, ~, ~, ~, ~] = setupProblem();
r = 0.10 * ones(7,1);

minThresh = 1.5 * r(1);

% downsample
% geom_xyz_ds = pcdownsample( pointCloud(geom_xyz'), 'random', 0.05).Location';
geom_xyz_ds = pcdownsample( pointCloud(geom_xyz'), 'random', 0.5).Location';
no_ds = size(geom_xyz_ds, 2);

geom_xyz = geom_xyz_ds;
no = no_ds;

q0 = reshape(q0, 7, 1);
% q0 = [0.6739, 0.7750, 1.4204, -0.3347, -0.1543, 0.6711, 0.0684]';
qg = [0.6739    1.4631    1.5522    1.3407   -0.1543    0.6711    0.0684]';

% qg_ = [];
% while true
%     qg = ik(eef, Tg, [1 1 1 1 1 1], robot.randomConfiguration);
%     c = collisionFcn_q(qg, geom_xyz, r);
%     coll = c >= 0;
%     length(find(coll))
%     if all(~coll)
%         break
%     end
% end

qg = repmat(qg, [1, Nmax]);

% q0 = 0.1 * randn(7,1) +  qg(:,1);

% q0 = robot.randomConfiguration;

% qg1 = robot.randomConfiguration;
% qg2 = robot.randomConfiguration;
% qg3 = robot.randomConfiguration;

% ng = 3;
% qg = repmat(qg3, [1, Nmax]);
% qg(:, 1 : floor(Nmax/ng)) = repmat(qg1, [1, floor(Nmax/ng)]);
% qg(:, ceil(Nmax/ng) : floor(2*Nmax/ng)) = repmat(qg2, [1, floor(2*Nmax/ng)-ceil(Nmax/ng)+1]);

% sq_idx = square(2*pi*0.005 * (1:Nmax));

A = eye(n); 
B = dt * eye(n);
% Q = 1e4 * eye(n);  
Q = 1e12 * eye(n);  
Ru = eye(n);  
P = Q;
[S, M, Qbar, Rbar, ~] = uncMPC(N, A, B, Q, Ru, P);
H = 2 * S' * Qbar * S + Rbar;
f0 = 2 * S' * Qbar * M;

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

% U_UB = 5 * U_UB;
% L_LB = 5 * U_LB;

% U_UB = [];
% L_LB = [];

% parpool
%% MPC
q = zeros(n, Nmax);     q(:,1) = q0;
u = zeros(n, Nmax);
U0 = zeros(N*n, 1);
loopTime = zeros(Nmax-1, 1);
ops0 = optimoptions('fmincon', 'SpecifyObjectiveGradient',true, ...
    'UseParallel', false, 'Display', 'iter', 'SpecifyConstraintGradient', true, ...
    'MaxIterations', 3000, 'MaxFunctionEvaluations', 10000);

% ops = optimoptions('fmincon', 'Algorithm', 'sqp', 'SpecifyObjectiveGradient',true, ...
%     'UseParallel', true, 'Display', 'iter', 'SpecifyConstraintGradient', true,...
%      'MaxIterations', 5);
 
ops = optimoptions('fmincon', 'Algorithm', 'sqp', 'SpecifyObjectiveGradient',true, ...
    'UseParallel', false, 'SpecifyConstraintGradient', true);

no_loop = 5;

for k = 1:Nmax-1
    tic
    % Update "initial state"
    q0 = q(:,k);
    % Update constraints
    Wtil = W + T * q0;
    % Update geom_xyz_ds
%     [dn, ~, ~] = get_dn_ddndq(q0, geom_xyz);
%     dn = min(reshape(dn, 7, no));
%     [dns,idxs] = sort(dn);
%     idxs2 = idxs(dns < minThresh);
%     geom_xyz_ds = [geom_xyz(:, idxs2), [100; 100; 100]];
%     no_ds = size(geom_xyz_ds,2);
%     mdl = min(dns)
%     md_loop(k) = mdl;
    [dn, ~, ~] = get_dn_ddndq(q0, geom_xyz);
    dn = min(reshape(dn, 7, no));
    [dns,idxs] = sort(dn);
    idxs2 = idxs(1:no_loop);
    geom_xyz_ds = [geom_xyz(:, idxs2), [100; 100; 100]];
    no_ds = size(geom_xyz_ds,2);
    mdl = min(dns)
    md_loop(k) = mdl;

    % Compute h
%     tform = getTransforms_iiwa7_mex(q0);
%     p = squeeze(tform(1:3, 4, :));
%     p17 = p(:, 1:7);
%     pjj = diff(p,1,2);
%     h = zeros(no_ds, 7);
%     for i = 1:no_ds
%         h(i,:) = dot(geom_xyz_ds(:,i) - p17, pjj) ./ dot(pjj, pjj);
%     end
    h = get_h(q0, geom_xyz_ds);
%     [c, ceq, gc, gceq] = collisionFcn_U_h(U0, geom_xyz_ds, r, S, M, q0, h);
    
    % Perform optimization
    if k == 1
        [Ustar, fval(k)] = fmincon(@(U) costFcn_q(U, S, M, Qbar, Rbar, q0, qg(:,k)), ...
                        U0, G, Wtil, [], [], U_LB, U_UB, ...
                        @(U) collisionFcn_U_h_mex(U, geom_xyz_ds, r, S, M, q0, h), ops0);
    else
        [Ustar, fval(k)] = fmincon(@(U) costFcn_q(U, S, M, Qbar, Rbar, q0, qg(:,k)), ...
                        U0, G, Wtil, [], [], U_LB, U_UB, ...
                        @(U) collisionFcn_U_h_mex(U, geom_xyz_ds, r, S, M, q0, h), ops);
    end
    fval(k)
    u(:,k) = Ustar(1:n);
    q(:,k+1) =  q(:,k) + dt * u(:,k);
%     U0 = Ustar;
    U0(1:end-n) = Ustar(n+1:end);
    U0(end-n+1:end) = U0(end-n); %U0(end-2*n+1: end-n);
    toc
    loopTime(k) = toc;
    disp(['%%%%%%%%%%%%%%%%%%%%%  iteration: ', num2str(k)])
end
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


%% mindist

for k = 1:Nmax
    [dn, ~, ~] = get_dn_ddndq(q(:,k), geom_xyz);
    md(k) = min(dn);
end

figure
plot(t, md);
yline(r(1));

%%
figure
pg = Tg(1:3, 4);
visualizeRobotTraj(robot, q, t, pg, geom_xyz, 0.05)
