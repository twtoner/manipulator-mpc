% clear;clc
% importKuka

% Test whether the formulation of pose as x = [w; x] matches the pose given
% by the Jacobian = dF/dq.

q = robot.randomConfiguration;
tic
J1 = getJacobians_iiwa7(q, 'eef');

dq = 0.001;
for i = 1:7
    dq_ = zeros(7,1);
    dq_(i) = dq;
    q_more = q + dq_;
    q_less = q - dq_;
    x_more = getPose_iiwa_eef(q_more);
    x_less = getPose_iiwa_eef(q_less);
    dxdq(:,i) = (x_more - x_less) / (2*dq);
end

J1 - dxdq

%%
q0 = robot.randomConfiguration;
q1 = robot.randomConfiguration;

T0 = getTransforms_iiwa7_eef(q0);
p0 = tform2trvec(T0)';      R0 = tform2rotm(T0);   
T1 = getTransforms_iiwa7_eef(q1);
p1 = tform2trvec(T1)';      R1 = tform2rotm(T1);   

zeta_hat = logm(T1 * T0^-1);
v = zeta_hat(1:3, 4);
what = zeta_hat(1:3, 1:3);

dt = 0.0001;
t = 0 : dt : 1;
m = length(t);

p = p0;
R = R0;
T = T0;
for i = 1:m
%     p(:,i+1) = p(:,i) + v * dt;
    R(:,:,i+1) = R(:,:,i) * expm(what * dt);
%     T(:,:,i+1) = [R(:,:,i+1), p(:,i+1); 0, 0, 0 1];\
    T(:,:,i+1) = expm(zeta_hat * dt) * T(:,:,i);
end
T(:,:,end) - T1




