clear;clc;
close all
importKuka

% Tracking end-effector pose x tracking with formulation 1 (translation
% only - 3 dimensions).

ik = inverseKinematics('RigidBodyTree', robot);

% dt = 0.01;
% Nmax = 100;
dt = 0.1;
Nmax = 50;
t = 0 : dt : (Nmax-1)*dt;
n = 7;
m = 3;
N = 20;

qg = robot.randomConfiguration;
% qg = [2.6372   -1.3696    1.8839   -1.0040   -2.9298    0.6511   -2.4196]';
xg = getPose_iiwa_eef(qg);
xg = xg(4:6);

% qg2 = robot.randomConfiguration;
% Tg2 = robot.getTransform(qg2, eef);
% pg2 = Tg2(1:3, 4);
% xg2 = getPose_iiwa_eef(qg2);

q0 = robot.randomConfiguration;
% q0 = [0.7707    1.3386    1.0898   -0.8300    1.2339    0.6723    2.5607]';
% TT = getTransforms_iiwa7(q0);   T0 = TT(:,:,end);
% p0 = T0(1:3,4);
x0 = getPose_iiwa_eef(q0);
x0 = x0(4:6);
% x0(1:3) = 0;

% freq = 0.5;
% xg = pg + 0.1 * sin(2*pi*freq*t);

% xg1 = repmat(pg, [1, ceil(Nmax/2)]);
% xg2 = repmat(pg2, [1, floor(Nmax/2)]);
% xg = [xg1, xg2];
% Xg = reshape(xg, Nmax*m, 1);

% Xg = repmat(pg, [N, 1]);
xg = repmat(xg, [1, Nmax]);

    
A = eye(n); 
B = dt * eye(n);
Q = eye(n);  
Ru = eye(n);  
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
Qx = 1e4 * diag([1 1 1]);
Px = 1 * Qx;
L = cell(1, N);
for i = 1:N-1
    L{i} = Qx;
end
L{N} = Px;
Qbar = blkdiag(L{:});

% cost = @(U) costFcn(U, S, M, Qbar, Rbar, q0, Xg);

%% MPC
% Determine how long a single cost function evaluation takes
ns = 10;
UU = rand(N*n, ns);
tic
for i = 1:ns
    [JJ, dJdU] = costFcn_X3(UU(:,i), S, M, Qbar, Rbar, q0, rand(m,1));
end
cfTime = toc/ns;
maxCfEvals = max(floor(dt / cfTime), 10);
% maxCfEvals = 10000;
%%
q = zeros(n, Nmax);     q(:,1) = q0;
u = zeros(n, Nmax);
ops0 = optimoptions('fmincon', 'SpecifyObjectiveGradient',true, 'UseParallel', false);
% ops1 = optimoptions('fmincon', 'Algorithm', 'sqp', 'SpecifyObjectiveGradient',true, 'MaxFunctionEvaluations', maxCfEvals);
ops1 = optimoptions('fmincon', 'Algorithm', 'sqp', 'SpecifyObjectiveGradient',true, 'UseParallel', false);
U0 = zeros(N*n, 1);
for i = 1:Nmax-1
    q0 = q(:,i);
    Wtil = W + T * q0;
%     cost = @(U) costFcn(U, S, M, Qbar, Rbar, q0, Xg);
    if i == 1
        [Ustar, fval(i)] = fmincon(@(U) costFcn_X3(U, S, M, Qbar, Rbar, q0, xg(:,i)), ...
                         U0, G, Wtil, [], [], U_LB, U_UB, [], ops0);
    else
        [Ustar, fval(i)] = fmincon(@(U) costFcn_X3(U, S, M, Qbar, Rbar, q0, xg(:,i)), ...
                         U0, G, Wtil, [], [], U_LB, U_UB, [], ops1);
    end
    u(:,i) = Ustar(1:n);
    q(:,i+1) =  q(:,i) + dt * u(:,i);
%     U0 = Ustar;
    U0(1:end-n) = Ustar(n+1:end);
    U0(end-n+1:end) = U0(end-n); 
    i
end

%%
% T = zeros(4,4,Nmax);
% for i = 1:Nmax
%     T(:,:,i) = getTransforms_iiwa7_eef(q(:,i));
% end
% p = squeeze(T(1:3,4,:));
% err = p - xg;
x = zeros(m, Nmax);
for i = 1:Nmax
    pp = getPose_iiwa_eef(q(:,i));
    x(:,i) = pp(4:6);
end
err = x - xg;

fig = figure;
% fig.Position = [1000 1200 500 1000];  % office PC
fig.Position = [3825 -300 1920 1080];     % home PC side monitor

subplot(3,1,1);
% plot(t, err', '*--'); 
% grid on
hold on
plot(t, x(1,:), 'r--')
plot(t, x(2,:), 'g--')
plot(t, x(3,:), 'b--')
plot(t, xg(1,:), 'r-', 'LineWidth', 2)
plot(t, xg(2,:), 'g-', 'LineWidth', 2)
plot(t, xg(3,:), 'b-', 'LineWidth', 2)
grid on
hold off
title('eef tracking - trans')

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


%% test gradient
u = randn(n*N,1);
q0 = randn(7,1);
[~, dJdU] = costFcn_X3(u, S, M, Qbar, Rbar, q0, xg(:,1));

dJdU2 = zeros(size(dJdU));
du = 1e-8;
for i = 1:n*N
    du_ = zeros(n*N,1);
    du_(i) = du;
    u_more = u + du_;
    u_less = u - du_;
    [J_more, ~] = costFcn_X3(u_more, S, M, Qbar, Rbar, q0, xg(:,1));
    [J_less, ~] = costFcn_X3(u_less, S, M, Qbar, Rbar, q0, xg(:,1));
    dJdU2(i) = (J_more - J_less) / (2*du);
end


max(abs(dJdU2 - dJdU), [], 'all')
