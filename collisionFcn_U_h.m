function [c, ceq, gradC, gradCeq] = collisionFcn_U_h(U, O, r, S, M, q0, h)
no = size(O, 2);
n = 7;
N = length(U)/n;
Q = S * U + M * q0;
q = reshape(Q, n, N);
r_ = repmat(r, no, 1);

dD_dQ = zeros(n*N*no, n*N);
C = zeros(n*N*no, 1);
for k = 1:N
    d_ = zeros(n*no, 1);
    d_dq = zeros(n*no, n);
    for i = 1:no
        dn = get_dn_3_mex(q(:,k), O(:,i));
        ddndq = get_ddndq_3_mex(q(:,k), O(:,i));
        dn1 = get_dn_1_mex(q(:,k), O(:,i));
        ddndq1 = get_ddndq_1_mex(q(:,k), O(:,i));
        dn2 = get_dn_2_mex(q(:,k), O(:,i));
        ddndq2 = get_ddndq_2_mex(q(:,k), O(:,i));
%         dn = get_dn_3(q(:,k), O(:,i));
%         ddndq = get_ddndq_3(q(:,k), O(:,i));
%         dn1 = get_dn_1(q(:,k), O(:,i));
%         ddndq1 = get_ddndq_1(q(:,k), O(:,i));
%         dn2 = get_dn_2(q(:,k), O(:,i));
%         ddndq2 = get_ddndq_2(q(:,k), O(:,i));
        for j = 1:7
            if h(i,j) > 1
                dn(j) = dn1(j);
                ddndq(:,j) = ddndq1(:,j);
            elseif h(i,j) < 0
                dn(j) = dn2(j);
                ddndq(:,j) = ddndq2(:,j);
            end
        end
%         [dn, ddndq, ~] = get_dn_ddndq(q(:,k), O(:,i));
        d_( (i-1)*n+1 : i*n ) = dn;
        d_dq( (i-1)*n+1 : i*n, :) = ddndq;
    end
    c_ = r_ - d_;
    dD_dQ( (k-1)*n*no+1 : k*n*no, (k-1)*n+1 : k*n) = d_dq;
%     dD_dQ_cell{k} = d_dq;
    C( (k-1)*n*no+1 : k*n*no )  = c_;
end

dD_dU = dD_dQ * S;
dC_dU = -dD_dU;

c = C;
gradC = dC_dU';

% Note: consider only the nearest X obstacles OR the obstalces closer than
% some threshold.

ceq = [];
gradCeq = [];