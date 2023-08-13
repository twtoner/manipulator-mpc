function [J, dJdU] = costFcn_q(U, S, M, Qbar, Rbar, q0, qg)
n = length(q0);
N = size(Qbar,1)/n;
Q = S*U + M*q0;
Qg = repmat(qg, N, 1);

J = (Q-Qg)' * Qbar * (Q-Qg) + U' * Rbar * U; % + (q0-qg)'*Qbar(1:n, 1:n)*(q0-qg);
dJdU = (2 * S' * Qbar * S + Rbar)*U + 2 * S' * Qbar * M * (q0 - qg);
% dJdU = H*U + fT;