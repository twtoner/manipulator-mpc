function x = getPose_iiwa_eef(q)

% T = getTransforms_iiwa7_eef(q);
T = getTransforms_iiwa7_eef_mex(q);
p = T(1:3, 4);
aa = tform2axang(T);
w = aa(1:3)' * aa(4);
% w = aa(1:3)';
% w = tform2eul(T, 'zyz')';

x = [w; p];