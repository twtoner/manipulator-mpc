function [J, dJdU] = costFcn_q_soft(U, O, S, M, Qbar, Rbar, q0, qg)
% Same as costFcn_q except also penalizing proximity to obstacles

n = length(q0);
N = size(Qbar,1)/n;
Q = S*U + M*q0;
Qg = repmat(qg, N, 1);

% Tracking
J1 = (Q-Qg)' * Qbar * (Q-Qg) + U' * Rbar * U; % + (q0-qg)'*Qbar(1:n, 1:n)*(q0-qg);
dJdU1 = (2 * S' * Qbar * S + Rbar)*U + 2 * S' * Qbar * M * (q0 - qg);

% Obstacles
fact = 1e10;
q = reshape(Q, 7, N);
J2 = 0;
dJdU2 = zeros(size(dJdU1));
for k = 1:N
    J2 = J2 + fact * getCF_H_exp(q(:,k), O);
    dJdU2( (k-1)*n+1 : k*n ) = fact * getCFgrad_dHdq_exp(q(:,k), O);
end

% s = zeros(1,N);
% for k = 1:N
%     T = getTransforms_iiwa7_mex(q(:,k));
%     p = squeeze(T(1:3, 4, :));
%     s(k) = 0;
%     for i = 1:no
%         s(k) = s(k) + sum(vecnorm(p - O(:,i)));
%     end
% end
% J2 = -sum(s);


J = J1 + J2;
dJdU = dJdU1 + dJdU2;

% dJdU = [];