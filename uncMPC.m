function [S, M, Qbar, Rbar, K0N] = uncMPC(N, A, B, Q, R, P)
%% Unconstrained MPC
% Problem 3

n = size(B, 1);
m = size(B, 2);

%% Build S
S = zeros(N*n, N*m);
S(1:n, 1:m) = B;
row = 1;
prev = B;
for i = 1 : N - 1
    next = [prev, A * prev(:, end-m+1:end)];
    ncols = size(next,2);
    row = row + m;
    S(row : row + m-1, 1:ncols) = next;
    prev = next;
end

%% Build M
M = zeros(N*n, n);
k = 1;
for i = 1:N
    M(k : k+n-1, :) = A^i;
    k = k + n;
end

%% Build Qbar
QQ = repmat(Q, N, 1);
Qcell = mat2cell(QQ, n*ones(1,N), n);
Qbar = blkdiag(Qcell{:});
Qbar(end-(n-1):end, end-(n-1):end) = P;

%% Build Rbar
RR = repmat(R, N, 1);
Rcell = mat2cell(RR, m*ones(1,N), m);
Rbar = blkdiag(Rcell{:});

%% Compute K0N
premult = zeros(m, m*N);
premult(1:m, 1:m) = -eye(m);
K0N = premult * (S' * Qbar * S + Rbar)^-1 * S' * Qbar * M;
