clear;clc
importKuka;

%% Symbolic gradient of Tvec wrt q
% qn = robot.randomConfiguration;
% q = sym('q', [7 1], 'real');
% T = getTransforms_iiwa7_sym(q);
% T = T(:,:,end);
% Tvec = TtoTvec(T);
% 
% dTdq = jacobian(Tvec, q);
% 
% dTdq_num = double(subs(dTdq, q, qn));
% T = sym('T', [4 4]);
% Tvec = TtoTvec(T)

%% Test dedq = dlogTdq = dlogTdT * dTdq
q = robot.randomConfiguration;
T = getTransforms_iiwa7_eef_mex(q);
Tg = eul2tform(randn(1,3)) * trvec2tform(rand(1,3));
Terr = inv(Tg) * T;
dTdq = get_dTdq(q, Tg);
dlogTdT = get_dlogTdT(Terr);

dlogTdq = dlogTdT * dTdq;

% Compare numerically
dq = 1e-6;
for i = 1:7
    dq_ = zeros(7,1);
    dq_(i) = dq;
    q_more = q + dq_;
    q_less = q - dq_;
    T_more = getTransforms_iiwa7_eef_mex(q_more);
    T_less = getTransforms_iiwa7_eef_mex(q_less);
    x_more = pseudolog(inv(Tg) * T_more);
    x_less = pseudolog(inv(Tg) * T_less);
    dxdq(:,i) = (x_more - x_less) / (2*dq);
end

max(abs(dxdq - dlogTdq), [], 'all')

%% Test J, dJdq
% q = robot.randomConfiguration;
% Tg = eul2tform(randn(1,3)) * trvec2tform(rand(1,3));

[~, dJdq] = costFcn_q_local(q, Tg);

dJdq_num = zeros(size(dJdq));
% Compare numerically
dq = 1e-6;
for i = 1:7
    dq_ = zeros(7,1);
    dq_(i) = dq;
    q_more = q + dq_;
    q_less = q - dq_;
    J_more = costFcn_q_local(q_more, Tg);
    J_less = costFcn_q_local(q_less, Tg);
    dJdq_num(i) = (J_more - J_less) / (2*dq);
end

max(abs(dJdq - dJdq_num), [], 'all')

%% Code for get_dTdq
q = sym('q', [7 1], 'real');
invTg = sym('invTg', [4 4], 'real');
T = getTransforms_iiwa7_sym(q);
T = T(:,:,end);
Tvec = TtoTvec(invTg * T);
dTdq = jacobian(Tvec, q);
% dTdq = double(subs(dTdq, [q; invTg(:)], [qn; invTgn(:)]));
matlabFunction(dTdq, 'File', 'get_dTdq')


%%
% q1 = robot.randomConfiguration;
% q2 = robot.randomConfiguration;
% 
% Ta = getTransforms_iiwa7_eef_mex(q1);
% Tb = getTransforms_iiwa7_eef_mex(q2);
% 
% xa = plog(Ta);
% xb = plog(Tb);
% 
% % relative transform
% b_T_a = inv(Tb) * Ta;
% 
% b_x_a = plog(b_T_a);
% 
% xa - xb;



% Replaced with code file
% function dTdq = get_dTdq(qn, Tgn)
% invTgn = inv(Tgn);
% q = sym('q', [7 1], 'real');
% invTg = sym('invTg', [4 4], 'real');
% T = getTransforms_iiwa7_sym(q);
% T = T(:,:,end);
% Tvec = TtoTvec(invTg * T);
% dTdq = jacobian(Tvec, q);
% dTdq = double(subs(dTdq, [q; invTg(:)], [qn; invTgn(:)]));
% end


function [J, dJdq] = costFcn_q_local(q, Tg)
Q = diag([1 1 1 1 1 1]);
T = getTransforms_iiwa7_eef_mex(q);
Terr = inv(Tg) * T;
e = plog(Terr);
J = e' * Q * e;
dJde = 2 * Q * e;
dedq = get_dlogTdT( Terr ) * get_dTdq( q, Tg ); % check

dJdq = dedq' * dJde;

end


