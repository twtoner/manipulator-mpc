clear;clc;close all
importKuka

dt = 1;
N = 2;
n = 7;
m = 3;
g = sym('g', [m, 1]);
Xg = repmat(g, [N, 1]);
q0 = sym('q0', [n, 1]);
syms rr qq
A = eye(n); 
B = dt * eye(n);
Q = eye(n);
R = eye(n);
[S, M, ~, Rbar, ~] = uncMPC(N, A, B, Q, R, Q);

Rbar = rr * Rbar;

syms dt
S = dt * S;

Qx = qq * eye(m);
L = cell(1, N);
for i = 1:N
    L{i} = Qx;
end
Qbar = blkdiag(L{:});

u = sym('u', [n, N]);
U = u(:);

[J, dJdU] = costFcn(U, S, M, Qbar, Rbar, q0, Xg);

H = sym('H', [N*n, N*n]);
for i = 1:N*n
    for j = 1:N*n
        H(i,j) = diff(dJdU(i), U(j));
    end
end
H = simplify(H);


%% Evaluate positive definiteness for random U's
ns = 10;
tic
eigs = zeros(N*n, ns);
for i = 1:ns
    Un = randn(size(U));
    gn = randn(size(g));
%     dtn = rand(size(dt));
    dtn = 0.01;
    rrn = rand(size(rr));
    qqn = rand(size(qq));
%     rrn = 1;
%     qqn = 1;
    q0n = robot.randomConfiguration;
    Hn = double(subs(H, [U; g; dt; rr; qq; q0], ...
                        [Un; gn; dtn; rrn; qqn; q0n] ));
    eigs(:,i) = eig(Hn);
end
toc/ns
min(eigs)

%%
function [J, dJdU] = costFcn(U, S, M, Qbar, Rbar, q0, Xg)

Q = S * U + M * q0;
n = 7;
m = 3;
N = length(Q) / n;
% Forward kinematics 
X = sym('X', [N*m,1]);
for i = 1:N
    q = Q( (i-1)*n+1 : (i)*n );
    TT = getTransforms_iiwa7_sym(q);
    X( (i-1)*m+1 : (i)*m ) = TT(1:3,4,end);
end 
% Jacobian block matrix
Jbar = getJac_bar(Q);
% Total cost gradient
dJdU = 2 * S.' * Jbar.' * Qbar * (X - Xg) + 2 * Rbar * U;
% Total cost
J = (X-Xg).' * Qbar * (X-Xg) + U.' * Rbar * U;
end

