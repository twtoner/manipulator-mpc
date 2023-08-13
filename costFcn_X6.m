function [J, dJdU] = costFcn_X6(U, S, M, Qbar, Rbar, q0, Tg)
Q = S * U + M * q0;
n = 7;
m = 6;
N = length(Q) / n;
% T = zeros(4,4,N);
% for i = 1:N
%     q = Q( (i-1)*n+1 : (i)*n );
%     T(:,:,i) = getTransforms_iiwa7_eef_mex(q);
% end
% % 
% Xdiff = getX6diff(T, Tg);
% % Jacobian block matrix
% Jbar = getJac_bar6(Q);
% % Total cost
% J = (Xdiff)' * Qbar * (Xdiff) + U' * Rbar * U;
% % Total cost gradient
% dJdU = 2 * S' * Jbar' * Qbar * (Xdiff) + 2 * Rbar * U;

% Tracking
invTg = inv(Tg);
J1 = 0;
dJdQ = zeros(N*m,1);
for i = 1:N
    q = Q( (i-1)*n+1 : (i)*n );
    T = getTransforms_iiwa7_eef_mex(q);
    Terr = invTg * T;
    e = pseudolog_mex(Terr);
    J1 = J1 + e' * Qbar( (i-1)*m+1 : i*m, (i-1)*m+1 : i*m ) * e;
    dJde = 2 * Qbar( (i-1)*m+1 : i*m, (i-1)*m+1 : i*m )  * e;
    dedq = get_dlogTdT_mex( Terr ) * get_dTdq_mex( q, Tg );
    dJdQ( (i-1)*n+1 : i*n ) = dedq' * dJde;
end
J = J1 + U' * Rbar * U;
dJdU = S' * dJdQ + 2 * Rbar * U;   

end
