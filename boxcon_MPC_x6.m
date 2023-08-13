clear;clc;
close all
importKuka

% Tracking end-effector pose x tracking with formulation 1 (translation and
% rotation - 6 dimensional).

ik = inverseKinematics('RigidBodyTree', robot);

% dt = 0.01;
% Nmax = 5;
dt = 0.1;
Nmax = 20;
t = 0 : dt : (Nmax-1)*dt;
n = 7;
m = 6;
N = 2;

qg = robot.randomConfiguration;
% qg = [2.6372   -1.3696    1.8839   -1.0040   -2.9298    0.6511   -2.4196]';
% xg = getPose_iiwa_eef(qg);
Tg = getTransforms_iiwa7_eef(qg);
% xg(1:3) = 0;

% qg2 = robot.randomConfiguration;
% Tg2 = robot.getTransform(qg2, eef);
% pg2 = Tg2(1:3, 4);
% xg2 = getPose_iiwa_eef(qg2);

q0 = robot.randomConfiguration;
% q0 = [0.7707    1.3386    1.0898   -0.8300    1.2339    0.6723    2.5607]';
% TT = getTransforms_iiwa7(q0);   T0 = TT(:,:,end);
% p0 = T0(1:3,4);
% x0 = getPose_iiwa_eef(q0);
T0 = getTransforms_iiwa7_eef(q0);
% x0(1:3) = 0;

% freq = 0.5;
% xg = pg + 0.1 * sin(2*pi*freq*t);

% xg1 = repmat(pg, [1, ceil(Nmax/2)]);
% xg2 = repmat(pg2, [1, floor(Nmax/2)]);
% xg = [xg1, xg2];
% Xg = reshape(xg, Nmax*m, 1);

% Xg = repmat(pg, [N, 1]);
% xg = repmat(xg, [1, Nmax]);
Tg = repmat(Tg, [1, 1, Nmax]);
    
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
Qx = 1e3 * diag([1 1 1 1 1 1]);
Px = 1 * Qx;
% Qx = 10 * eye(m);
% Px = Qx;
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
    [JJ, dJdU] = costFcn_X6(UU(:,i), S, M, Qbar, Rbar, q0, eul2tform(rand(1,3)));
end
cfTime = toc/ns;
maxCfEvals = max(floor(dt / cfTime), 10);
% maxCfEvals = 10000;
%%
q = zeros(n, Nmax);     q(:,1) = q0;
u = zeros(n, Nmax);
ops0 = optimoptions('fmincon', 'SpecifyObjectiveGradient',false, 'UseParallel', true);
% ops1 = optimoptions('fmincon', 'Algorithm', 'sqp', 'SpecifyObjectiveGradient',false, 'UseParallel', true, 'MaxFunctionEvaluations', maxCfEvals);
ops1 = optimoptions('fmincon', 'Algorithm', 'sqp', 'SpecifyObjectiveGradient',false, 'UseParallel', true, ...
        'MaxIterations', 1000);
U0 = zeros(N*n, 1);
for i = 1:Nmax-1
    q0 = q(:,i);
    Wtil = W + T * q0;
%     cost = @(U) costFcn(U, S, M, Qbar, Rbar, q0, Xg);
    if i == 1
        [Ustar, fval(i)] = fmincon(@(U) costFcn_X6(U, S, M, Qbar, Rbar, q0, Tg(:,:,i)), ...
                         U0, G, Wtil, [], [], U_LB, U_UB, [], ops0);
    else
        [Ustar, fval(i)] = fmincon(@(U) costFcn_X6(U, S, M, Qbar, Rbar, q0, Tg(:,:,i)), ...
                         U0, G, Wtil, [], [], U_LB, U_UB, [], ops1);
    end
    u(:,i) = Ustar(1:n);
    q(:,i+1) =  q(:,i) + dt * u(:,i);
%     U0 = Ustar;

    U0(1:end-n) = Ustar(n+1:end);
    U0(end-n+1:end) = U0(end-n); 
    %U0(end-2*n+1: end-n);
    i
    fval(i)
end

%%
% T = zeros(4,4,Nmax);
% for i = 1:Nmax
%     T(:,:,i) = getTransforms_iiwa7_eef(q(:,i));
% end
% p = squeeze(T(1:3,4,:));
% err = p - xg;
% x = zeros(m, Nmax);

err = zeros(6, Nmax);
for i = 1:Nmax
%     x(:,i) = getPose_iiwa_eef(q(:,i));
    T = getTransforms_iiwa7_eef(q(:,i));
    err(:,i) = getX6diff(T, Tg(:,:,i));
end
% err = x - xg;

fig = figure;
% fig.Position = [1000 1200 500 1000];  % office PC
% fig.Position = [3825 -300 1920 1080];     % home PC side monitor

subplot(4,1,1);
% plot(t, err', '*--'); 
% grid on
hold on
plot(t, err(1,:), 'r--')
plot(t, err(2,:), 'g--')
plot(t, err(3,:), 'b--')
% plot(t, xg(1,:), 'r-', 'LineWidth', 2)
% plot(t, xg(2,:), 'g-', 'LineWidth', 2)
% plot(t, xg(3,:), 'b-', 'LineWidth', 2)
grid on
hold off
title('eef tracking - rot')

subplot(4,1,2);
% plot(t, err', '*--'); 
% grid on
hold on
plot(t, err(4,:), 'r--')
plot(t, err(5,:), 'g--')
plot(t, err(6,:), 'b--')
% plot(t, xg(4,:), 'r-', 'LineWidth', 2)
% plot(t, xg(5,:), 'g-', 'LineWidth', 2)
% plot(t, xg(6,:), 'b-', 'LineWidth', 2)
grid on
hold off
title('eef tracking - trans')


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


%% test gradient
% u = randn(n*N,1);
% q0 = randn(7,1);
% Tg = Tg(:,:,1);
% [~, dJdU] = costFcn_X6(u, S, M, Qbar, Rbar, q0, Tg);
% 
% dJdU2 = zeros(size(dJdU));
% du = 1e-6;
% for i = 1:n*N
%     du_ = zeros(n*N,1);
%     du_(i) = du;
%     u_more = u + du_;
%     u_less = u - du_;
%     [J_more, ~] = costFcn_X6(u_more, S, M, Qbar, Rbar, q0, Tg);
%     [J_less, ~] = costFcn_X6(u_less, S, M, Qbar, Rbar, q0, Tg);
%     dJdU2(i) = (J_more - J_less) / (2*du);
% end
% 
% 
% max(abs(dJdU2 - dJdU), [], 'all')
% 
% %% check Jacobian of FK as [w; dx]
% q = robot.randomConfiguration;
% T = getTransforms_iiwa7_eef_mex(q);
% J = getJacobians_iiwa7_eef_mex(q);
% P = getPose_iiwa_eef(q);
% 
% dq = 1e-8;
% for i = 1:7
%     dq_ = zeros(7,1);
%     dq_(i) = dq;
%     q_more = q + dq_;
%     q_less = q - dq_;
%     P_more = getPose_iiwa_eef(q_more);
%     P_less = getPose_iiwa_eef(q_less);
%     dif = getX6diff(getTransforms_iiwa7_eef_mex(q_less), getTransforms_iiwa7_eef_mex(q_more));
% %     dPdq(:,i) = (P_more - P_less)/(2*dq);
%     dPdq(:,i) = dif/(2*dq);
% end


%%

