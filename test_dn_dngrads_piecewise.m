clear;clc
importKuka;

%% 
% Test dn and dn grads for each test case

%% Case 1

q = robot.randomConfiguration;
o = randn(3,1);

dn = get_d_1(q, o);
ddndq_sym = get_dddq_1(q, o);

ddndq_num = zeros(size(ddndq_sym));
dq = 1e-6;
for i = 1:7
    dq_ = zeros(7,1);   dq_(i) = dq;
    q_more = q + dq_;
    q_less = q - dq_;
    dn_more = get_d_1(q_more, o);
    dn_less = get_d_1(q_less, o);
    ddndq_num(:,i) = (dn_more - dn_less) / (2*dq);
end

maxerr1 = max(abs(ddndq_sym - ddndq_num), [], 'all');
disp(['Max error for case 1: ', num2str(maxerr1)]);
    
%% Case 2

q = robot.randomConfiguration;
o = randn(3,1);

dn = get_d_2(q, o);
ddndq_sym = get_dddq_2(q, o);

ddndq_num = zeros(size(ddndq_sym));
dq = 1e-6;
for i = 1:7
    dq_ = zeros(7,1);   dq_(i) = dq;
    q_more = q + dq_;
    q_less = q - dq_;
    dn_more = get_d_2(q_more, o);
    dn_less = get_d_2(q_less, o);
    ddndq_num(:,i) = (dn_more - dn_less) / (2*dq);
end
    
maxerr2 = max(abs(ddndq_sym - ddndq_num), [], 'all');
disp(['Max error for case 2: ', num2str(maxerr2)]);
  
%% Case 3

q = robot.randomConfiguration;
o = randn(3,1);

dn = get_d_3(q, o);
ddndq_sym = get_dddq_3(q, o);

ddndq_num = zeros(size(ddndq_sym));
dq = 1e-6;
for i = 1:7
    dq_ = zeros(7,1);   dq_(i) = dq;
    q_more = q + dq_;
    q_less = q - dq_;
    dn_more = get_d_3(q_more, o);
    dn_less = get_d_3(q_less, o);
    ddndq_num(:,i) = (dn_more - dn_less) / (2*dq);
end
    
maxerr3 = max(abs(ddndq_sym - ddndq_num), [], 'all');
disp(['Max error for case 3: ', num2str(maxerr3)]);

% Conclusion -> all cases are correct!