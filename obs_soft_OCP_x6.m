clear;clc;
close all
importKuka

% Tracking end-effector pose x tracking with formulation 1 (6 dimensions). 
% Now considers soft constraints and hard constraints

ik = inverseKinematics('RigidBodyTree', robot);

% dt = 0.01;
% Nmax = 100;
dt = 0.1;
Nmax = 20;
t = 0 : dt : (Nmax-1)*dt;
n = 7;
m = 6;
% N = 20;
N = Nmax;

% [geom_xyz, q0, no, ~, ~, Tg, ~, ~, ~, ~, ~, ~, ~, ~, ~] = setupProblem();
wall = 0.1;
% Box center and inside dimensions
bc = [0.75, 0, 0.5];
bdi = [0.6, 1.0, 0.4];
bdo = bdi + wall*[0 1 1];
geom_xyz_inner = generate_geometry(bdi, bc, [-1, 0, 0], 0.05);  
geom_xyz_outer = generate_geometry(bdo, bc, [-1, 0, 0], 0.05);
geom_xyz = [geom_xyz_inner, geom_xyz_outer];
pg = [0.75; 0.1; 0.5];
Ry = eul2rotm([0, pi/2, 0]);
Rz = eul2rotm([0, 0, pi/2]);
Rg = Rz * Ry;
Tg = [Rg, pg; 0, 0, 0, 1];
q0 = [0.3559   -0.5249    0.0939   -2.0591    1.6915   -0.4401   -0.0814]';
% q0 = [0   0    0.0939   -2.0591    1.6915   -0.4401   -0.0814]';
% q0(4) = q0(4) - 0.4;
% q0(1) = q0(1) + 0.1;

r = 0.10 * ones(7,1);
% downsample
geom_xyz_ds = pcdownsample( pointCloud(geom_xyz'), 'gridAverage', 0.12).Location';
no_ds = size(geom_xyz_ds, 2);

% T0 = Tg;
% q0 = ik(eef, Tg, [1 1 1 1 1 1], zeros(7,1));

A = eye(n); 
B = dt * eye(n);
Q = eye(n);  
Ru = 1 * eye(n);  
P = Q;
[S, M, ~, Rbar, ~] = uncMPC(N, A, B, Q, Ru, P);

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

% Cost on X
% Qx = 30 * eye(m);
% Qx = 1e1 * diag([1 1 1]);
% Px = 1e12 * Qx;
% Qx = 1e10 * diag([1 1 1 1 1 1]);
Qx = 1000000 * eye(m);
Px = 1 * Qx;
L = cell(1, N);
for i = 1:N-1
    L{i} = Qx;
end
L{N} = Px;
Qbar = blkdiag(L{:});

%% OCP

ops = optimoptions('fmincon', 'Algorithm', 'interior-point', ...
        'SpecifyObjectiveGradient', false, 'UseParallel', true, ...
        'SpecifyConstraintGradient', true, ...
        'Display', 'iter', 'PlotFcn', {'optimplotfvalconstr', 'optimplotx', 'optimplotfval'}, ...
        'MaxIterations', 300, 'MaxFunctionEvaluations', 100000);

% if exist('U_prev')
%     U0 = U_prev;
% else
U0 = zeros(N*n, 1);
dHdq = getCFgrad_dHdq_exp(q0, geom_xyz_ds);
NM = getNullSpaceMat_iiwa7(q0);
u0 = -NM * dHdq;
u0 = -dHdq;
[mx, idx] = max(u0);
U0(1:n) = u0;
% end
    
tic
[c, ceq, gradC, gradCeq] = collisionFcn_U_mex(U0, geom_xyz_ds, r, S, M, q0);
toc

tic
[J, dJdU] = costFcn_X6_soft(U0, geom_xyz_ds, S, M, Qbar, Rbar, q0, Tg(:,:,1));
toc

tic
[Ustar, fval, exitflag] = fmincon(@(U) costFcn_X6_soft(U, geom_xyz_ds, S, M, Qbar, Rbar, q0, Tg(:,:,1)), ...
                U0, G, Wtil, [], [], U_LB, U_UB, ...
                @(U) collisionFcn_U_mex(U, geom_xyz_ds, r, S, M, q0), ops);
% [Ustar, fval, exitflag] = fmincon(@(U) costFcn_X6_soft(U, geom_xyz_ds, S, M, Qbar, Rbar, q0, Tg(:,:,1)), ...
%                 U0, G, Wtil, [], [], U_LB, U_UB, ...
%                 [], ops);
toc

[c, ceq, gradC, gradCeq] = collisionFcn_U_mex(Ustar, geom_xyz_ds, r, S, M, q0);


Q = S*Ustar + M*q0;
u = reshape(Ustar, n, []);
q = reshape(Q, n, []);

U_prev = Ustar;

%% MPC
% % Determine how long a single cost function evaluation takes
% ns = 10;
% UU = rand(N*n, ns);
% tic
% for i = 1:ns
%     [JJ, dJdU] = costFcn_X3(UU(:,i), S, M, Qbar, Rbar, q0, rand(m,1));
% end
% cfTime = toc/ns;
% maxCfEvals = max(floor(dt / cfTime), 10);
% % maxCfEvals = 10000;
% %
% q = zeros(n, Nmax);     q(:,1) = q0;
% u = zeros(n, Nmax);
% ops0 = optimoptions('fmincon', 'SpecifyObjectiveGradient',true, 'UseParallel', false);
% % ops1 = optimoptions('fmincon', 'Algorithm', 'sqp', 'SpecifyObjectiveGradient',true, 'MaxFunctionEvaluations', maxCfEvals);
% ops1 = optimoptions('fmincon', 'Algorithm', 'sqp', 'SpecifyObjectiveGradient',true, 'UseParallel', false);
% U0 = zeros(N*n, 1);
% for i = 1:Nmax-1
%     q0 = q(:,i);
%     Wtil = W + T * q0;
% %     cost = @(U) costFcn(U, S, M, Qbar, Rbar, q0, Xg);
%     if i == 1
%         [Ustar, fval(i)] = fmincon(@(U) costFcn_X3(U, S, M, Qbar, Rbar, q0, xg(:,i)), ...
%                          U0, G, Wtil, [], [], U_LB, U_UB, [], ops0);
%     else
%         [Ustar, fval(i)] = fmincon(@(U) costFcn_X3(U, S, M, Qbar, Rbar, q0, xg(:,i)), ...
%                          U0, G, Wtil, [], [], U_LB, U_UB, [], ops1);
%     end
%     u(:,i) = Ustar(1:n);
%     q(:,i+1) =  q(:,i) + dt * u(:,i);
%     U0 = Ustar;
%     U0(1:end-n) = Ustar(n+1:end);
%     U0(end-n+1:end) = U0(end-n); 
%     i
% end

%%
% for i = 1:Nmax
%     err = getTransforms_iiwa7_eef(q(:,i));
% end
% p = squeeze(T(1:3,4,:));
% err = p - xg;
err = zeros(m, Nmax);
T = zeros(4,4,Nmax);
for i = 1:Nmax
    T(:,:,i) = getTransforms_iiwa7_eef(q(:,i));
    err(:,i) = getX6diff(T(:,:,i), Tg);
end

fig = figure;
% fig.Position = [1000 1200 500 1000];  % office PC
fig.Position = [3825 -300 1920 1080];     % home PC side monitor

subplot(4,1,1);
% plot(t, err', '*--'); 
% grid on
hold on
plot(t, err(1,:), 'r--')
plot(t, err(2,:), 'g--')
plot(t, err(3,:), 'b--')
grid on
hold off
title('eef error - rot')

subplot(4,1,2);
% plot(t, err', '*--'); 
% grid on
hold on
plot(t, err(4,:), 'r--')
plot(t, err(5,:), 'g--')
plot(t, err(6,:), 'b--')
grid on
hold off
title('eef error - trans')

subplot(4,1,3)
hold on
plot(t, q', '*--');
yline(q_UB, 'r--');
yline(q_LB, 'b--');
hold off
grid on
title('joint traj.')

subplot(4,1,4)
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

md = zeros(1,Nmax);
for k = 1:Nmax
    [dn, ~, ~] = get_dn_ddndq(q(:,k), geom_xyz_ds);
    md(k) = min(dn);
end

figure
plot(t, md);
yline(r(1));


%%
pg = Tg(1:3, 4);
visualizeRobotTraj(robot, q, t, pg, geom_xyz_ds, 1)

%% test gradient
% geom_xyz_ds = rand(3,1);
% u = randn(n*N,1);
% q0 = randn(7,1);
% [~, dJdU] = costFcn_X3_soft(u, geom_xyz_ds, S, M, Qbar, Rbar, q0, xg(:,1));
% 
% dJdU2 = zeros(size(dJdU));
% du = 1e-6;
% for i = 1:n*N
%     du_ = zeros(n*N,1);
%     du_(i) = du;
%     u_more = u + du_;
%     u_less = u - du_;
%     [J_more, ~] = costFcn_X3_soft(u_more, geom_xyz_ds, S, M, Qbar, Rbar, q0, xg(:,1));
%     [J_less, ~] = costFcn_X3_soft(u_less, geom_xyz_ds, S, M, Qbar, Rbar, q0, xg(:,1));
%     dJdU2(i) = (J_more - J_less) / (2*du);
% end
% 
% 
% percent_err = 100 * (dJdU - dJdU2) ./ dJdU;
% 
% percent_err = percent_err(percent_err < 99);    % filter out 100% values where subtracting ~0 - ~0 and dividing by small 2*dq

