clear;clc;close all
importKuka

% obs = generate_geometry([0.7, 1.0, 0.3], [0.72, 0.25, 0.4], [-1, 0, 0], 0.05);  
dt = 0.01;
m = 50;
n = 7;

pg = [0.75; 0; 0.4];
Rg = eul2rotm([0, pi/2, 0]);
Tg = [Rg, pg; 0, 0, 0, 1];
q0 = [0.3559   -0.5249    0.0939   -2.0591    1.6915   -0.4401   -0.0814]';

q_UB = [2.9671 2.0944 2.9671 2.0944 2.9671 2.0944 3.0543]';
q_LB = -q_UB;

Q_UB = repmat(q_UB, [m, 1]);
Q_LB = repmat(q_LB, [m, 1]);

u_UB = (pi/180) * [98 98 100 130 140 180 180]';
u_LB = -u_UB;

U_UB = repmat(u_UB, [m, 1]);
U_LB = repmat(u_LB, [m, 1]);

A = eye(n); B = dt * eye(n);
Q = eye(n);  Ru = 0.001 * eye(n);  P = eye(n);
[S, M, Qbar, Rbar, ~] = uncMPC(m, A, B, Q, Ru, P);



U0 = randn(n*m, 1);

ops = optimoptions('fmincon', 'PlotFcn', 'optimplotfval', 'MaxFunctionEvaluations', 1e6);
[Ustar, fval, exitflag, output] = fmincon(@(U)costFcn(U, q0, Tg, dt), U0, [], [], [], [], U_LB, U_UB, [], ops)

% Closed form (unconstrained)
Ustar2 = 

%%
ustar = reshape(Ustar, n, m);
q = q0 + dt * cumsum(ustar,2);
x = zeros(3,m);
for i = 1:m
    TT = getTransforms_iiwa7(q(:,i));
    T = TT(:,:,end);
    p = T(1:3,4);
    x(:,i) = p;
end

figure; plot(ustar', '*--'); title('ustar');
figure; plot(q', '*--'); title('joint traj from ustar')
figure; plot((x-pg)', '*--'); title('position error')


function J = costFcn(U, q0, Tg, dt)
% Q = eye(3);
% R = eye(7);

pg = Tg(1:3,4);
% Rg = Tg(1:3,1:3);
n = length(q0);
m = length(U)/n;
u = reshape(U, n, []);
q = q0 + dt * cumsum(u, 2);
x = zeros(3,m);
for i = 1:m
    TT = getTransforms_iiwa7(q(:,i));
    T = TT(:,:,end);
    p = T(1:3,4);
%     R = T(1:3,1:3);
    x(:,i) = p;
end

Jx = 10000 * sum(vecnorm(x-pg).^2);
Ju = sum(vecnorm(u).^2);

J = Jx + Ju;

end