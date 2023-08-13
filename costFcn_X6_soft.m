function [J, dJdU] = costFcn_X6_soft(U, O, S, M, Qbar, Rbar, q0, Tg)
Q = S * U + M * q0;
n = 7;
m = 6;
N = length(Q) / n;
% Xg = repmat(xg, [N, 1]);
% Forward kinematics 
% X = zeros(N*m,1);
% for i = 1:N
%     q = Q( (i-1)*n+1 : (i)*n );
% %     TT = getTransforms_iiwa7(q);
% %     X( (i-1)*m+1 : (i)*m ) = TT(1:3,4,end);
%     X( (i-1)*m+1 : (i)*m ) = getPose_iiwa_eef(q);
% end 
T = zeros(4,4,N);
for i = 1:N
    q = Q( (i-1)*n+1 : (i)*n );
    T(:,:,i) = getTransforms_iiwa7_eef_mex(q);
end
% 
Xdiff = getX6diff(T, Tg);
% Jacobian block matrix
% Jbar = getJac_bar6(Q);

% Tracking
% cost
J1 = (Xdiff)' * Qbar * (Xdiff) + U' * Rbar * U;
% gradient
% dJdU1 = 2 * S' * Jbar' * Qbar * (Xdiff) + 2 * Rbar * U;

% Obstacles
% fact = 1e8;
% fact = 1;   % good for J2 = -dn
fact = 1000;
% fact = 1e-3;
% fact = 0;
% no = size(O,2);
q = reshape(Q, 7, N);
% dJdU2 = zeros(size(dJdU1));
% dn = zeros(N, 1);
% eta = 1;
% beta = 0.1;
J2 = 0;
for k = 1:N
%     for i = 1:no
%         dd = get_dn_2_mex(q(:,k), O(:,i));
%         for j = 1:7
%             J2 = J2 + eta * (dd(j)/beta - 1)^2;
%         end
%     end
    J2 = J2 + fact * getCF_H_exp(q(:,k), O);
%     dJdU2( (k-1)*n+1 : k*n ) = fact * getCFgrad_dHdq_exp(q(:,k), O);
%     dn(k) = min(vecnorm(getDIJ_proj(q(:,k), O)), [], 'all');
end
% J2 = -min(dn);
J = J1 + J2;
% dJdU = dJdU1 + dJdU2;
dJdU = [];
% disp([J1 J2])
% [dn, ~, ~] = get_dn_ddndq(q(:,end), O);
% J1
% J2
% disp(['Mindist: ', num2str(min(dn))])
% endState = Xdiff(end-5:end);
% rot = endState(1:3);    tran = endState(4:6);
% disp(['Final trans error norm: ', num2str(norm(rot))])
% disp(['Final rot error norm: ', num2str(norm(tran))])
% disp('------------');
end
