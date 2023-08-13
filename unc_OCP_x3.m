clear;clc;
close all
importKuka

% Tracking X where X is position only (3 dimensional). 

ik = inverseKinematics('RigidBodyTree', robot);

MANUAL = false;

dt = 0.01;
N = 100;
t = 0 : dt : (N-1)*dt;
n = 7;
m = 3;

% qg = robot.randomConfiguration;
qg = [2.6372   -1.3696    1.8839   -1.0040   -2.9298    0.6511   -2.4196]';
Tg = robot.getTransform(qg, eef);
pg = Tg(1:3, 4);    Rg = Tg(1:3, 1:3);

% q0 = robot.randomConfiguration;
q0 = [0.7707    1.3386    1.0898   -0.8300    1.2339    0.6723    2.5607]';
TT = getTransforms_iiwa7(q0);   T0 = TT(:,:,end);
p0 = T0(1:3,4);
Xg = repmat(pg, [N, 1]);

% alpha = linspace(0, 1, N);
% for i = 1:N
%     q_guess(:,i) = alpha(i) * qg + (1 - alpha(i)) * q0;
% end
% u_guess = gradient(q_guess)/dt;
% U_guess = reshape(u_guess, [N*n, 1]);

A = eye(n); 
B = dt * eye(n);
Q = eye(n);  
Ru = eye(n);  
P = Q;
[S, M, ~, Rbar, ~] = uncMPC(N, A, B, Q, Ru, P);

% Cost on X
Qx = 1000 * eye(m);
L = cell(1, N);
for i = 1:N
    L{i} = Qx;
end
Qbar = blkdiag(L{:});

cost = @(U) costFcn_X3(U, S, M, Qbar, Rbar, q0, Xg);

%% Manual gradient descent
if MANUAL
    tic
    U0 = zeros(N*n, 1);
    ns = 10000;
    U = zeros(N*n, ns);
    gamma = 100e-3;
    J = zeros(1,ns-1);
    dJdU = zeros(N*n, ns-1);
    for i = 1:ns-1
        [J(i), dJdU(:,i)] = cost(U(:,i));
        U(:,i+1) = U(:,i) - gamma * dJdU(:,i);
    end
    figure; plot(J); title('cost'); xlabel('iterations')
    Ustar = U(:,end);
    toc
end
%% fminunc
U0 = zeros(N*n,1);
ops = optimoptions('fminunc','Algorithm','quasi-newton','SpecifyObjectiveGradient',true);
tic
Ustar = fminunc(cost, U0, ops);
toc

%%
u = reshape(Ustar, n, N);
q = q0 + dt * cumsum(u,2);
T = zeros(4,4,N);
for i = 1:N
    TT = getTransforms_iiwa7(q(:,i));
    T(:,:,i) = TT(:,:,end);
end
p = squeeze(T(1:3,4,:));
err = p - pg;

fig = figure;
fig.Position = [1000 1200 500 1000];
subplot(3,1,1);
plot(t, err', '*--'); 
grid on
title('eef error')

subplot(3,1,2)
plot(t, q', '*--');
grid on
title('joint traj.')

subplot(3,1,3)
plot(t, u', '*--');
grid on
title('control input')

disp(['final error: ', num2str(err(:,end)')]);
disp(['error norm: ', num2str(norm(err(:,end)))]);

%%

