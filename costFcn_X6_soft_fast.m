function [J, dJdU] = costFcn_X6_soft_fast(U, O, S, M, Qbar, Rbar, q0, Tg)
Q = S * U + M * q0;
n = 7;
m = 6;
N = length(Q) / n;

% Tracking
invTg = inv(Tg);
J1 = 0;
dJdQ = zeros(N*m,1);
for i = 1:N
    q = Q( (i-1)*n+1 : (i)*n );
    T = getTransforms_iiwa7_eef_mex(q);
    Terr = invTg * T;
    e = pseudolog(Terr);
    J1 = J1 + e' * Qbar( (i-1)*m+1 : i*m, (i-1)*m+1 : i*m ) * e;
    dJde = 2 * Qbar( (i-1)*m+1 : i*m, (i-1)*m+1 : i*m )  * e;
    dedq = get_dlogTdT( Terr ) * get_dTdq( q, Tg );
    dJdQ( (i-1)*n+1 : i*n ) = dedq' * dJde;
end
J1 = J1 + U' * Rbar * U;
dJdU1 = S' * dJdQ + 2 * Rbar * U;   

% Obstacles
fact = 1000;
J2 = 0;
dJdU2 = zeros(N*n, 1);
for k = 1:N
    q = Q( (i-1)*n+1 : (i)*n );
    J2 = J2 + fact * getCF_H_exp(q, O);
    dJdU2( (k-1)*n+1 : k*n ) = fact * getCFgrad_dHdq_exp(q, O);
end
J = J1 + J2;
dJdU = dJdU1 + dJdU2;
end
