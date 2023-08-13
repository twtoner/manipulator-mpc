function c = collisionFcn_q(q, O, r)

D = getDIJ_proj(q, O);
cc1 = r - squeeze(vecnorm(D));
c = cc1(:);

