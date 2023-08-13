clear;clc;
close all
importKuka

ik = inverseKinematics('RigidBodyTree', robot);


MANUAL = false;

dt = 0.01;
N = 30;
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
Qx = 1000 * eye(m);
L = cell(1, N);
for i = 1:N
    L{i} = Qx;
end
Qbar = blkdiag(L{:});

cost = @(U) costFcn_X3(U, S, M, Qbar, Rbar, q0, Xg);

%% fmincon
U0 = zeros(N*n,1);
% U0 = Ustar + 0.1 * randn(size(Ustar));
% ops = optimoptions('fmincon','Algorithm', 'sqp', 'SpecifyObjectiveGradient',true);
ops = optimoptions('fmincon','SpecifyObjectiveGradient',true,'UseParallel',false, 'MaxFunctionEvaluations', 10);
tic
Ustar = fmincon(cost, U0, G, Wtil, [], [], U_LB, U_UB, [], ops);
toc

% U0 = Ustar + 0.05 * randn(size(Ustar));
% tic
% Ustar = fmincon(cost, U0, G, Wtil, [], [], U_LB, U_UB, [], ops);
% toc


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

fig = figure
fig.Position = [1000 1200 500 1000];
subplot(3,1,1);
plot(t, err', '*--'); 
grid on
title('eef error')

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
% ns = 100;
% tic
% U0 = rand(N*n, ns);
% for i = 1:ns
%     [JJ, dJdUU] = cost(U0(:,i));
% end
% toc/ns

%%
function [J, dJdU] = costFcn(U, S, M, Qbar, Rbar, q0, Xg)

Q = S * U + M * q0;
n = 7;
m = 3;
N = length(Q) / n;
% Forward kinematics 
X = zeros(N*m,1);
for i = 1:N
    q = Q( (i-1)*n+1 : (i)*n );
    TT = getTransforms_iiwa7(q);
    X( (i-1)*m+1 : (i)*m ) = TT(1:3,4,end);
end 
% Jacobian block matrix
Jbar = getJac_bar(Q);
% Total cost gradient
dJdU = 2 * S' * Jbar' * Qbar * (X - Xg) + 2 * Rbar * U;
% Total cost
J = (X-Xg)' * Qbar * (X-Xg) + U' * Rbar * U;
end

