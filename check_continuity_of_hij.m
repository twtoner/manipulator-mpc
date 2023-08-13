clear
clc
close all
importKuka

% Check that the piecewise definition of hij is actually continuous

z = 0 : 0.01 : 1;
x = 0.25 * ones(size(z));
y = 0.25 * ones(size(z));
o = [x; y; z];
% o = [0.25; 0.25; 0.6];
q = robot.homeConfiguration;
dq = 1e-4;
q2 = q + dq * randn(7,1);


[dn, ~, h] = get_dn_ddndq(o, q)

dn = reshape(dn, 7, []);

figure
plot(o(3,:), dn')
legend('L1', 'L2', 'L3', 'L4', 'L5', 'L6', 'L7');
xlabel('obstacle position')
ylabel('|| dij ||')

figure
hold on
robot.show(q, 'PreservePlot', true, 'Visuals', 'off');
plot3dpts(o);
view([90 0])
grid on
hold off

% results - check!