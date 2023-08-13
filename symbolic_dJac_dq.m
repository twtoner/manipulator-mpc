clear;clc;close all
importKuka

q = sym('q', [7, 1]);

J = getJacobians_iiwa7(q, 'eef');

dJdq = sym('dJdq', [6, 7, 7]);
for i = 1:6             % row
    for j = 1:7         % col
        for k = 1:7     % page
            dJdq(i,j,k) = simplify(diff( J(i,j), q(k) ));
        end
    end
end

