clear;clc
importKuka;

%
% Testing get_ddndq_3, get_ddndq_3_v2, and get_ddndq_3_vpa

rand(3,100);
clc

no = 1;

O = randn(3, no);

q = robot.randomConfiguration;

tic
ddndq3_1 = get_dddq_3(q, O);
t1 = toc;

tic
ddndq3_2 = get_dddq_3_v2(q, O);
t2 = toc;

tic
ddndq3_3 = get_dddq_3_vpa(q, O);
t3 = toc;

s
[t1 t2 t3]


