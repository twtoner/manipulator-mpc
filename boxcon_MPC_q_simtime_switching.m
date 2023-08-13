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

% q0 = robot.randomConfiguration;
q0 = [2.2032    0.4759   -1.3563    1.4851   -2.0327   -1.0077    1.0420]';

qg1 = [-2.8772    0.9005   -1.6700    0.7905   -1.5072   -0.9798   -2.2476]';
qg2 = [0.6864   -1.0125    0.7780    0.8190   -0.5717   -1.2839   -1.1785]';
qg3 = [1.1450    0.5460   -1.4846   -1.9125    0.1866   -1.3563    0.1338]';
% qg1 = robot.randomConfiguration;
% qg2 = robot.randomConfiguration;
% qg3 = robot.randomConfiguration;

ng = 3;
qg = repmat(qg3, [1, Nmax_sim]);
qg(:, 1 : floor(Nmax_sim/ng)) = repmat(qg1, [1, floor(Nmax_sim/ng)]);
qg(:, ceil(Nmax_sim/ng) : floor(2*Nmax_sim/ng)) = repmat(qg2, [1, floor(2*Nmax_sim/ng)-ceil(Nmax_sim/ng)+1]);

A = eye(n); 
B = dt * eye(n);
Q = 1e1 * eye(n);  
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

%% MPC
q = zeros(n, Nmax_sim);
q(:,1) = q0;
u = zeros(n, Nmax_sim);
U0 = zeros(N*n, 1);
k = 1;
mpcTime = zeros(mpcTimeFactor,1);
for i = 1:Nmax_sim-1
    
    if (mpcTimeFactor==1) || (mod(i, mpcTimeFactor) == 1)   % do MPC at i = 1, 11, 21, ...
        tic
        f = f0 * (q(:,i) - qg(:,i));
        Ustar = quadprog(H, f, G, Wtil, [], [], U_LB, U_UB);
        % Shift (unnecessary since initial guess U0 is ignored by default)
%         if N > 1
%             U0(1:end-n) = Ustar(n+1:end);
%             U0(end-n+1:end) = U0(end-n); 
%         end
        % Assign next control move
        uu = Ustar(1:n);
        % MPC time
        mpcTime(k) = toc;
        k = k + 1;
    end
    % Simulate
    u(:,i) = uu;
    q(:,i+1) = q(:,i) + u(:,i) * dt_sim;
    i
end

%%
err = q - qg;

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
for i = 1:7
    plot(t_sim, err(i,:), '-', 'Color', jntColors{i})
end
hold off
grid on
title('Switching Joint Goal Tracking Error $q_k - q_{g,k}$: Switching Static Goal', 'Interpreter', 'latex')
xlabel('time (s)', 'Interpreter', 'latex')
ylabel('angle error (rad)', 'Interpreter', 'latex'); 
l = legend('$q_{1,k}$', '$q_{2,k}$', '$q_{3,k}$', '$q_{4,k}$', '$q_{5,k}$', '$q_{6,k}$', '$q_{7,k}$',...
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
ylim([-0.05, 0.15])
hold off
grid on
xlabel('time (s)', 'Interpreter', 'latex')
ylabel('MPC time (s)', 'Interpreter', 'latex'); 
title('MPC Computation Time', 'Interpreter', 'latex')
set(gca,'TickLabelInterpreter','latex')


f = gcf
exportgraphics(f,'figures/boxcon_MPC_q_simtime_switching.png','Resolution',500)


disp(['final error: ', num2str(err(:,end)')]);
disp(['error norm: ', num2str(norm(err(:,end)))]);

disp(['mean mpc time: ', num2str(mean(mpcTime))])

