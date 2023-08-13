function Xdiff = getX6diff(T, Tg)

% aa = tform2axang(T);
% R = tform2rotm(T);
% p = tform2trvec(T);
% 
% % aag = tform2axang(Tg);
% Rg = tform2rotm(Tg);
% pg = tform2trvec(Tg);

Terr = pagemtimes(Tg^-1, T);

% trans = p' - pg';
trans_err = tform2trvec(Terr)';
% R_err = tform2trvec(Terr);
% R_err = pagemtimes(Rg', R);

aa = tform2axang(Terr)';

rot_err = aa(4,:) .* aa(1:3,:);

Xdiff = [rot_err; trans_err];
Xdiff = Xdiff(:);
