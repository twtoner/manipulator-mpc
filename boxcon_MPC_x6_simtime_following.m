clear;clc;
close all
importKuka

% Joint q tracking with formulation 1.

dt_sim = 0.01;
Tmax = 15;
Nmax_sim = Tmax / dt_sim;
t_sim = 0 : dt_sim : (Nmax_sim-1)*dt_sim;

mpcTimeFactor = 10;
dt = dt_sim * mpcTimeFactor;
Nmax = Nmax_sim / mpcTimeFactor;
t = 0 : dt : (Nmax-1)*dt;

n = 7;  % Number of joints
N = 10;  % MPC horizon
m = 6;  % pose dimension

% q0 = robot.randomConfiguration;
% q0 = [2.2032    0.4759   -1.3563    1.4851   -2.0327   -1.0077    1.0420]';
q0 = zeros(7,1);

% Design reference trajectory
f = 0.25;
ax = 0.6;
ay = 0.1;
az = 0.1;

yy = ay * cos(2*pi*f*t_sim);
zz = az * sin(2*pi*f*t_sim) + 0.5;
xx = ax * ones(size(yy));
pg = [xx; yy; zz];
Ry = eul2rotm([0, pi/2, 0]);
Rz = eul2rotm([0, 0, pi/2]);
Rg = Rz * Ry;

for i = 1:Nmax_sim
    Tg(:,:,i) = [Rg, pg(:,i); 0, 0, 0 1];
end

A = eye(n); 
B = dt * eye(n);
Q = -2e1 * eye(n);   % doesn't matter
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

% Cost on X
Qx = 3e1 * diag([1 1 1 1 1 1]);
Px = 1 * Qx;
L = cell(1, N);
for i = 1:N-1
    L{i} = Qx;
end
L{N} = Px;
Qbar = blkdiag(L{:});

%% MPC
ops0 = optimoptions('fmincon', 'SpecifyObjectiveGradient',true, 'UseParallel', false);
% ops1 = optimoptions('fmincon', 'Algorithm', 'sqp', 'SpecifyObjectiveGradient',false, 'UseParallel', true, 'MaxFunctionEvaluations', maxCfEvals);
ops1 = optimoptions('fmincon', 'Algorithm', 'sqp', 'SpecifyObjectiveGradient',true, 'UseParallel', false, ...
        'MaxIterations', 1000);

q = zeros(n, Nmax_sim);
q(:,1) = q0;
u = zeros(n, Nmax_sim);
U0 = zeros(N*n, 1);
k = 1;
mpcTime = zeros(mpcTimeFactor,1);
for i = 1:Nmax_sim-1
    
    if (mpcTimeFactor==1) || (mod(i, mpcTimeFactor) == 1)   % do MPC at i = 1, 11, 21, ...
        tic
        if i == 1
            [Ustar, fval(i)] = fmincon(@(U) costFcn_X6(U, S, M, Qbar, Rbar, q(:,i), Tg(:,:,i)), ...
                             U0, G, Wtil, [], [], U_LB, U_UB, [], ops0);
        else
            [Ustar, fval(i)] = fmincon(@(U) costFcn_X6(U, S, M, Qbar, Rbar, q(:,i), Tg(:,:,i)), ...
                             U0, G, Wtil, [], [], U_LB, U_UB, [], ops1);
        end
        % Shift 
        if N > 1
            U0(1:end-n) = Ustar(n+1:end);
            U0(end-n+1:end) = U0(end-n); 
        end
        % Assign next control move
        uu = Ustar(1:n);
        % MPC time
        mpcTime(k) = toc;
        k = k + 1;
        
        i
    end
    % Simulate
    u(:,i) = uu;
    q(:,i+1) = q(:,i) + u(:,i) * dt_sim;
end

%%
perr = zeros(3, Nmax_sim);
werr = zeros(3, Nmax_sim);
for i = 1:Nmax_sim
    T = getTransforms_iiwa7_eef_mex(q(:,i));
    Terr = inv(Tg(:,:,i)) * T;
    perr(:,i) = Terr(1:3,4);
    aa = tform2axang(Terr);
    werr(:,i) = aa(4) * aa(1:3)';
end

err = [perr; werr];
    
fig = figure;
% fig.Position = [1000 1200 500 1000];  % office PC
% fig.Position = [3825 -300 1920 1080];     % home PC side monitor
fig.Position = [200 200 500 600];     % printing to figure
% 

% jntColors = ['y', 'm', 'c', 'r', 'g', 'b', 'w', 'k'];

jntColors = {[0    0.4471    0.7412], 
             [0.8510    0.3255    0.0980],
             [0.9294    0.6941    0.1255],
             [0.4941    0.1843    0.5569],
             [0.4667    0.6745    0.1882],
             [0.3020    0.7451    0.9333],
             [0.6353    0.0784    0.1843]};

subplot(4,1,1);
hold on
for i = 1:6
    plot(t_sim, err(i,:), '-', 'Color', jntColors{i})
end
hold off
grid on
title('Pose Reference Tracking Error $\texttt{plog}({}^gT_k)$ via Direct Pose Tracking', 'Interpreter', 'latex')
xlabel('time (s)', 'Interpreter', 'latex')
ylabel('angle error (rad)', 'Interpreter', 'latex'); 
l = legend('$e_{1,k}$', '$e_{2,k}$', '$e_{3,k}$', '$e_{4,k}$', '$e_{5,k}$', '$e_{6,k}$', '$e_{7,k}$',...
    'Interpreter', 'latex', 'location', 'east');
l.NumColumns = 1;

set(gca,'TickLabelInterpreter','latex')


subplot(4,1,2)
hold on
for i = 1:7
    plot(t_sim, q(i,:), '-', 'Color', jntColors{i})
    yline(q_UB(i), '--', 'Color', jntColors{i});
    yline(q_LB(i), '--', 'Color', jntColors{i});
end
hold off
grid on
xlabel('time (s)', 'Interpreter', 'latex')
ylabel('angle (rad)', 'Interpreter', 'latex'); 
title('Joint Trajectory $q_k$', 'Interpreter', 'latex')
set(gca,'TickLabelInterpreter','latex')

subplot(4,1,3)
hold on
for i = 1:7
    plot(t_sim, u(i,:), '-', 'Color', jntColors{i})
    yline(u_UB(i), '--', 'Color', jntColors{i});
    yline(u_LB(i), '--', 'Color', jntColors{i});
end
hold off
grid on
xlabel('time (s)', 'Interpreter', 'latex')
ylabel('angular rate (rad/s)', 'Interpreter', 'latex'); 
title('Control Input $u_k$', 'Interpreter', 'latex')
set(gca,'TickLabelInterpreter','latex')

subplot(4,1,4)
hold on
plot(t, mpcTime);
yline(dt)
ylim([-0.05, 0.2])
hold off
grid on
xlabel('time (s)', 'Interpreter', 'latex')
ylabel('MPC time (s)', 'Interpreter', 'latex'); 
title('MPC Computation Time', 'Interpreter', 'latex')
set(gca,'TickLabelInterpreter','latex')


f = gcf
exportgraphics(f,'figures/boxcon_MPC_x6_simtime_following.png','Resolution',500)

disp(['final error: ', num2str(err(:,end)')]);
disp(['error norm: ', num2str(norm(err(:,end)))]);

disp(['mean mpc time: ', num2str(mean(mpcTime))])


%%
% pg = Tg(1:3,4,end);
% visualizeRobotTraj(robot, q, t_sim, pg, [100; 100; 100], 1)
