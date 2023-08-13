clear;clc

N = 5;
n = 7;
m = 6;
dt = 0.1;
A = eye(n); 
B = dt * eye(n);
Q = eye(n);  
Ru = eye(n);  
P = 10 * Q;
[S, M, ~, Rbar, ~] = uncMPC(N, A, B, Q, Ru, P);
Qx = 1000000 * diag([1 1 1 1 1 1]);
Px = 1 * Qx;
L = cell(1, N);
for i = 1:N-1
    L{i} = Qx;
end
L{N} = Px;
Qbar = blkdiag(L{:});

Tg = eul2tform(rand(1,3)) * trvec2tform(rand(1,3));
q0 = randn(7,1);

no = 5;
O = randn(3, no);
U = randn(N*n, 1);

[~, dJdU] = costFcn_X6_soft_fast(U, O, S, M, Qbar, Rbar, q0, Tg);

dJdU_num = zeros(size(dJdU));
dU = 1e-6;
for i = 1:N*n
    dU_ = zeros(N*n,1);
    dU_(i) = dU;
    U_more = U + dU_;
    U_less = U - dU_;
    [J_more, ~] = costFcn_X6_soft_fast(U_more, O, S, M, Qbar, Rbar, q0, Tg);
    [J_less, ~] = costFcn_X6_soft_fast(U_less, O, S, M, Qbar, Rbar, q0, Tg);
    dJdU_num(i) = (J_more - J_less) / (2*dU);
end

perc_err = 100 * abs(dJdU - dJdU_num) ./ dJdU_num;
max(perc_err)

    
    
    
    
    
    
    