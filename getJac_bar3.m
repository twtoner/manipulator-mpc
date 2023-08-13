function Jbar = getJac_bar3(Q)
% Jacobian for X translation only (3-dimensional).
n = 7;

N = size(Q,1)/n;

L = cell(1, N);
for i = 1:N
    q = Q( (i-1)*n+1 : (i)*n);
    J = getJacobians_iiwa7_eef_mex(q);
    J = J(4:6,:); % only linear velocity
    L{i} = J;
end

Jbar = blkdiag( L{:} );



