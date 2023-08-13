clear;clc
importKuka

%%
[geom_xyz, q0, no, ~, ~, Tg, ~, ~, ~, ~, ~, ~, ~, ~, ~] = setupProblem();

% no = no;
% 
% q0 = robot.randomConfiguration
% 
% geom_xyz = randn(3,no);

tic
DIJ = getDIJ_proj(q0, geom_xyz);
t1 = toc;
%%
tic
T = getTransforms_iiwa7(q0);
p = squeeze(T(1:3, 4, :)); 
O = geom_xyz;

d = zeros(3, 7, no);
h = nan(no, 7);
P = nan(3,3,7);

for j = 1:7
    J = j+1;
    pjj = p(:,J)-p(:,J-1);
    h(:,j) = (O - p(:,J-1))' * pjj / (pjj' * pjj);
    P(:,:,j) = (pjj*pjj')/(pjj'*pjj) - eye(3);
    for i = 1:no
        if h(i,j) > 1
            d(:,j,i) = p(:,J) - O(:,i);
        elseif h(i,j) < 0 
            d(:,j,i) = p(:,J-1) - O(:,i);
        else
            d(:,j,i) = P(:,:,j) * (O(:,i) - p(:, J));
        end
    end
end

% max(dn1 - dn2, [], 'all')
t2 = toc;

%% Finally compare to new function
q = q0';
tic
[dn3, ~, h3] = get_dn_ddndq(O, q);
t3 = toc;

dn1 = squeeze(vecnorm(DIJ));
dn2 = squeeze(vecnorm(d));

max(dn1, [], 'all')
max(dn2, [], 'all')
max(dn3, [], 'all')

[t1, t2, t3]


%% Compare gradients
n=7;
no = 2;
O = randn(3,no);

% q = robot.randomConfiguration;
[~, grad_sym, hh] = get_dn_ddndq(O, q);

grad_num = zeros(n*no,n);
dq = 1e-3;
for i = 1:7
    dq_ = zeros(n,1);
    dq_(i) = dq;
    q_more = q + dq_;
    q_less = q - dq_;
    [dn_more, ~, h_more] = get_dn_ddndq(O, q_more);
    [dn_less, ~, h_less] = get_dn_ddndq(O, q_less);
%     norm(dn_more - dn_less)
    grad_num(:,i) = (dn_more - dn_less) / (2*dq);
    h_grad_num(:,:,i) = (h_more - h_less) / (2*dq);
end

% (grad_num - grad_sym) 

% err = max(abs(grad_num - grad_sym), [], 'all')
[mx, idx] = max(abs(grad_num - grad_sym), [], 'all', 'linear')

[r,c] = ind2sub(size(grad_num), idx)

% grad_sym
% grad_num
% hh

% h_grad_num
% grad_num

%% Compare gradients on the same "piece" of the piecewise hij

q = zeros(7,1);

no = 1;
O = [0.25; 0.25; 0.45] + 0.01 * randn(3,no);

[~, grad_sym, hh] = get_dn_ddndq(O, q);

grad_num = zeros(n*no,n);
h_grad_num = zeros(no, n, n);
dq = 1e-3;
for i = 1:7
    dq_ = zeros(n,1);
    dq_(i) = dq;
    q_more = q + dq_;
    q_less = q - dq_;
    [dn_more, ~, h_more] = get_dn_ddndq(O, q_more);
    [dn_less, ~, h_less] = get_dn_ddndq(O, q_less);
%     norm(dn_more - dn_less)
    grad_num(:,i) = (dn_more - dn_less) / (2*dq);
    h_grad_num(:,:,i) = (h_more - h_less) / (2*dq);
    any((h_more > 1) - (h_less > 1))
    any((h_more < 1) - (h_less < 1))
    
end

[mx, idx] = max(abs(grad_num - grad_sym), [], 'all', 'linear');

mx

[r,c] = ind2sub(size(grad_num), idx);


%% Compare gradients on the same "piece" of the piecewise hij

