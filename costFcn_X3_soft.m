function [J, dJdU] = costFcn_X3_soft(U, O, S, M, Qbar, Rbar, q0, xg)
% Cost function for tracking X when X describes translation only (3-dim).
% Now includes soft constraints on obstacles.
Q = S * U + M * q0;
n = 7;
m = 3;
N = length(Q) / n;
% Forward kinematics 
X = zeros(N*m,1);
for i = 1:N
    q = Q( (i-1)*n+1 : (i)*n );
    T = getTransforms_iiwa7_eef_mex(q);
    X( (i-1)*m+1 : (i)*m ) = T(1:3,4);
end
% Jacobian block matrix
Jbar = getJac_bar3(Q);

% Tracking
% tracking cost gradient
Xg = repmat(xg, [N, 1]);
dJdU1 = 2 * S' * Jbar' * Qbar * (X - Xg) + 2 * Rbar * U;
% tracking cost
J1 = (X-Xg)' * Qbar * (X-Xg) + U' * Rbar * U;

% Obstacles
fact = 1;
q = reshape(Q, 7, N);
J2 = 0;
dJdU2 = zeros(size(dJdU1));
for k = 1:N
    J2 = J2 + fact * getCF_H_exp(q(:,k), O);
    dJdU2( (k-1)*n+1 : k*n ) = fact * getCFgrad_dHdq_exp(q(:,k), O);
end

J = J1 + J2;
dJdU = dJdU1 + dJdU2;

end
