clear;clc
importKuka;

%%
% Test collisionFcn_U gradient output

% First test each case individually without considering closest (i.e. value
% of hij)

no = 22;
O = randn(3,no);

r = 0.09 * ones(7,1);

N = 10;
n = 7;
dt = 0.01;
A = eye(n); 
B = dt * eye(n);
Ru = eye(n);  
Q = 1e14 * eye(n);  
P = Q;
[S, M, Qbar, Rbar, ~] = uncMPC(N, A, B, Q, Ru, P);
q0 = robot.randomConfiguration;

U = 10 * randn(N*n, 1);
Q = S * U + M * q0;
q = reshape(Q, n, []);
%% Case 1

tic
[C, dC_dU] = build_C(U, O, S, M, q0, r, @get_d_1, @get_dddq_1);
toc

% Estimate numerically

dC_dU_num = zeros(size(dC_dU));
dU = 1e-6;
for i = 1:n*N
    dU_ = zeros(n*N, 1);
    dU_(i) = dU;
    U_more = U + dU_;
    U_less = U - dU_;
    [C_more, ~] = build_C(U_more, O, S, M, q0, r, @get_d_1, @get_dddq_1);
    [C_less, ~] = build_C(U_less, O, S, M, q0, r, @get_d_1, @get_dddq_1);
    dC_dU_num(:,i) = (C_more - C_less)/(2*dU);
end

err1 = max(abs(dC_dU - dC_dU_num), [], 'all')


%% Case 2

tic
[C, dC_dU] = build_C(U, O, S, M, q0, r, @get_d_2, @get_dddq_2);
toc

% Estimate numerically

dC_dU_num = zeros(size(dC_dU));
dU = 1e-6;
for i = 1:n*N
    dU_ = zeros(n*N, 1);
    dU_(i) = dU;
    U_more = U + dU_;
    U_less = U - dU_;
    [C_more, ~] = build_C(U_more, O, S, M, q0, r, @get_d_2, @get_dddq_2);
    [C_less, ~] = build_C(U_less, O, S, M, q0, r, @get_d_2, @get_dddq_2);
    dC_dU_num(:,i) = (C_more - C_less)/(2*dU);
end

err2 = max(abs(dC_dU - dC_dU_num), [], 'all')

%% Case 3

tic
[C, dC_dU] = build_C(U, O, S, M, q0, r, @get_d_3, @get_dddq_3);
toc

% Estimate numerically

dC_dU_num = zeros(size(dC_dU));
dU = 1e-6;
for i = 1:n*N
    dU_ = zeros(n*N, 1);
    dU_(i) = dU;
    U_more = U + dU_;
    U_less = U - dU_;
    [C_more, ~] = build_C(U_more, O, S, M, q0, r, @get_d_3, @get_dddq_3);
    [C_less, ~] = build_C(U_less, O, S, M, q0, r, @get_d_3, @get_dddq_3);
    dC_dU_num(:,i) = (C_more - C_less)/(2*dU);
end

err3 = max(abs(dC_dU - dC_dU_num), [], 'all')


%%
function [C, dC_dU] = build_C(U, O, S, M, q0, r, d_fcn, dd_fcn)
Q = S * U + M * q0;
no = size(O, 2);
n = 7;
N = length(Q)/n;
q = reshape(Q, n, []);
r_ = repmat(r, no, 1);

C = zeros(N*n*no, 1);
dD_dQ = zeros(N*n*no, N*n);

for k = 1:N
    d_ = zeros(n*no, 1);
    d_dq = zeros(n*no, n);
    for i = 1:no
        dn = d_fcn(q(:,k), O(:,i));
        ddndq = dd_fcn(q(:,k), O(:,i));
        d_( (i-1)*n+1 : i*n ) = dn;
        d_dq( (i-1)*n+1 : i*n, :) = ddndq;
    end
    c_ = r_ - d_;
    dD_dQ( (k-1)*n*no+1 : k*n*no, (k-1)*n+1 : k*n) = d_dq;
    C( (k-1)*n*no+1 : k*n*no )  = c_;
end

dD_dU = dD_dQ * S;
dC_dU = -dD_dU;

end