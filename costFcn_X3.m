function [J, dJdU] = costFcn_X3(U, S, M, Qbar, Rbar, q0, xg)
% Cost function for tracking X when X describes translation only (3-dim).
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
% Total cost gradient
Xg = repmat(xg, [N, 1]);
dJdU = 2 * S' * Jbar' * Qbar * (X - Xg) + 2 * Rbar * U;
% Total cost
J = (X-Xg)' * Qbar * (X-Xg) + U' * Rbar * U;
end
