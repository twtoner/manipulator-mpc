clear;clc;
close all
importKuka

% Tracking end-effector pose x tracking with formulation 1 (6 dimensions). 
% Now considers soft constraints and hard constraints.
% Considers that simulation runs at 100 Hz while MPC runs at 10 Hz.

ik = inverseKinematics('RigidBodyTree', robot);

dt_sim = 0.01;
Tmax = 2;
Nmax_sim = Tmax / dt_sim;
t_sim = 0 : dt_sim : (Nmax_sim-1)*dt_sim;

mpcTimeFactor = 10;
dt = dt_sim * mpcTimeFactor;
Nmax = Nmax_sim / mpcTimeFactor;
t = 0 : dt : (Nmax-1)*dt;

n = 7;
m = 6;
N = 3;

wall = 0.1;
% Box center and inside dimensions
bc = [0.75, 0, 0.5];
bdi = [0.6, 1.0, 0.4];
bdo = bdi + wall*[0 1 1];
geom_xyz_inner = generate_geometry(bdi, bc, [-1, 0, 0], 0.05);  
geom_xyz_outer = generate_geometry(bdo, bc, [-1, 0, 0], 0.05);
geom_xyz = [geom_xyz_inner, geom_xyz_outer];
no = size(geom_xyz, 2);
pg = [0.75; 0.1; 0.6];
% pg = bc';
Ry = eul2rotm([0, pi/2, 0]);
Rz = eul2rotm([0, 0, pi/2]);
% Rg = Rz * Ry;
Rg = eye(3);
Tg = [Rg, pg; 0, 0, 0, 1];
q0 = [0.3559   -0.5249    0.0939   -2.0591    1.6915   -0.4401   -0.0814]';

r = 0.10 * ones(7,1);
% global downsample
geom_xyz_ds_glo = pcdownsample( pointCloud(geom_xyz'), 'gridAverage', 0.12).Location';
no_ds_glo = size(geom_xyz_ds_glo, 2);

A = eye(n); 
B = dt * eye(n);
Q = eye(n);  
Ru = 1 * eye(n);  
P = Q;
[S, M, ~, Rbar, ~] = uncMPC(N, A, B, Q, Ru, P);

% Constraints
u_UB = (pi/180) * [98 98 100 130 140 180 180]';
u_LB = -u_UB;
U_UB = repmat(u_UB, [N, 1]);
U_LB = repmat(u_LB, [N, 1]);
q_UB = [2.9671 2.0944 2.9671 2.0944 2.9671 2.0944 3.0543]';
q_LB = -q_UB;
Q_UB = repmat(q_UB, [N, 1]);
Q_LB = repmat(q_LB, [N, 1]);
G = [S; -S; eye(size(S)); -eye(size(S))];
W = [Q_UB; -Q_LB; U_UB; -U_LB];
T = [-M; M; zeros(size(M)); zeros(size(M))];
Wtil = W + T * q0;

% Cost on X
Qx = 1000000 * diag([1 1 1 1 1 1]);
Px = 1 * Qx;
L = cell(1, N);
for i = 1:N-1
    L{i} = Qx;
end
L{N} = Px;
Qbar = blkdiag(L{:});

%% MPC
ops0 = optimoptions('fmincon', 'SpecifyObjectiveGradient',true, ...
            'SpecifyConstraintGradient', true, 'UseParallel', false, 'MaxIterations', 1000);
ops1 = optimoptions('fmincon', 'Algorithm', 'sqp', 'SpecifyObjectiveGradient',true, ...
            'SpecifyConstraintGradient', true, 'UseParallel', false, 'MaxIterations', 1000);

thresh_loop = 0.15; % obstacles closer than threshold
        
q = zeros(n, Nmax_sim);
q(:,1) = q0;
u = zeros(n, Nmax_sim);
U0 = zeros(N*n, 1);
mpcTime = zeros(mpcTimeFactor,1);
no_ds = zeros(mpcTimeFactor, 1);
k = 1;
for i = 1:Nmax_sim-1
    
    if (mpcTimeFactor==1) || (mod(i, mpcTimeFactor) == 1)   % do MPC at i = 1, 11, 21, ...
        tic
        % Local pointcloud downsample
        dn = min(squeeze(vecnorm(getDIJ_proj(q(:,i), geom_xyz))));
        idxs2 = find(dn < thresh_loop);
        geom_xyz_ds = [geom_xyz(:, idxs2), [0; 0; 100]];    % add faraway point in case idxs2 is empty
        no_ds(k) = size(geom_xyz_ds,2);
        if i == 1
            ops_ = ops0;
        else
            ops_ = ops1;
        end
        [Ustar, fval(i)] = fmincon(@(U) costFcn_X6_soft_fast(U, geom_xyz_ds_glo, S, M, Qbar, Rbar, q(:,i), Tg(:,:,1)), ...
                U0, G, Wtil, [], [], U_LB, U_UB, ...
                @(U) collisionFcn_U_mex(U, geom_xyz_ds, r, S, M, q(:,i)), ops1);
        
        % Shift
        if N > 1
            U0(1:end-n) = Ustar(n+1:end);
            U0(end-n+1:end) = U0(end-n); 
        end
        
        % Assign next control move
        uu = Ustar(1:n);
        
        % MPC time
        mpcTime(k) = toc;
        k = k + 1;
    end
    
    u(:,i) = uu;
    q(:,i+1) = q(:,i) + u(:,i) * dt_sim;
    i
end


% %% MPC
% q = zeros(n, Nmax);     q(:,1) = q0;
% u = zeros(n, Nmax);
% 
% no_loop = 20;   % x closest obstacles to consider
% thresh_loop = 0.15; % obstacles closer than threshold
% 
% U0 = zeros(N*n, 1);
% for i = 1:Nmax-1
%     q0 = q(:,i);
%     % Select closest obstacles
%     dn = min(squeeze(vecnorm(getDIJ_proj(q0, geom_xyz))));
%     idxs2 = find(dn < thresh_loop);
% %     [dns,idxs] = sort(dn);
% %     idxs2 = idxs(1:no_loop);
% %     geom_xyz_ds = geom_xyz(:, idxs2);
%     geom_xyz_ds = [geom_xyz(:, idxs2), [0; 0; 100]];    % add faraway point in case idxs2 is empty
%     no_ds = size(geom_xyz_ds,2)
%     % 
% %     [dd, ~, ~] = get_dn_ddndq(q0, geom_xyz_ds);
% %     dn(i) = min(dd);
% %     Tnow = getTransforms_iiwa7_eef_mex(q0);
% %     errNow = getX6diff(Tnow, Tg);
% %     rot = errNow(1:3);    tran = errNow(4:6);
% %     disp(['current trans error norm: ', num2str(norm(tran))])
% %     disp(['current rot error norm: ', num2str(norm(rot))])
% %     disp(['current mindist: ', num2str(dn(i))])
% %     disp('------------');
%     Wtil = W + T * q0;
%     if i > 1
%         [Ustar, fval(i)] = fmincon(@(U) costFcn_X6_soft_fast(U, geom_xyz_ds_glo, S, M, Qbar, Rbar, q0, Tg(:,:,1)), ...
%                 U0, G, Wtil, [], [], U_LB, U_UB, ...
%                 @(U) collisionFcn_U_mex(U, geom_xyz_ds, r, S, M, q0), ops1);
%     else
%         [Ustar, fval(i)] = fmincon(@(U) costFcn_X6_soft_fast(U, geom_xyz_ds_glo, S, M, Qbar, Rbar, q0, Tg(:,:,1)), ...
%                 U0, G, Wtil, [], [], U_LB, U_UB, ...
%                 @(U) collisionFcn_U_mex(U, geom_xyz_ds, r, S, M, q0), ops0);
%     end
% %     uu = Ustar(1:n);
% %     for j = 1:10
% %         q(:,k+1) = q(:,k) + uu * dt_sim;
% %         k = k + 1;
% %     end
%     u(:,i) = Ustar(1:n);
%     q(:,i+1) =  q(:,i) + dt * u(:,i);
%     U0 = Ustar;
%     if N > 1
%         U0(1:end-n) = Ustar(n+1:end);
%         U0(end-n+1:end) = U0(end-n); 
%     end
%     i
% end

%% mindist

md = zeros(1,Nmax);
md_ds = zeros(1,Nmax);
for k = 1:Nmax
    [dn, ~, ~] = get_dn_ddndq(q(:,k), geom_xyz);
    [dn_ds, ~, ~] = get_dn_ddndq(q(:,k), geom_xyz_ds_glo);
    md(k) = min(dn);
    md_ds(k) = min(dn_ds);
end

%%
perr = zeros(3, Nmax_sim);
werr = zeros(3, Nmax_sim);
for i = 1:Nmax_sim
    T = getTransforms_iiwa7_eef_mex(q(:,i));
    Terr = inv(Tg) * T;
    perr(:,i) = Terr(1:3,4);
    aa = tform2axang(Terr);
    werr(:,i) = aa(4) * aa(1:3)';
end

err = [perr; werr];
    
fig = figure;
% fig.Position = [1000 1200 500 1000];  % office PC
% fig.Position = [3825 -300 1920 1080];     % home PC side monitor
fig.Position = [200 200 500 800];     % printing to figure
% 

% jntColors = ['y', 'm', 'c', 'r', 'g', 'b', 'w', 'k'];

jntColors = {[0    0.4471    0.7412], 
             [0.8510    0.3255    0.0980],
             [0.9294    0.6941    0.1255],
             [0.4941    0.1843    0.5569],
             [0.4667    0.6745    0.1882],
             [0.3020    0.7451    0.9333],
             [0.6353    0.0784    0.1843]};

subplot(5,1,1);
hold on
for i = 1:6
    plot(t_sim, err(i,:), '-', 'Color', jntColors{i})
end
hold off
grid on
title('Pose Reference Tracking Error $\texttt{plog}({}^gT_k)$ via Pose Tracking w/ Obstacles', 'Interpreter', 'latex')
xlabel('time (s)', 'Interpreter', 'latex')
ylabel('angle error (rad)', 'Interpreter', 'latex'); 
l = legend('$e_{1,k}$', '$e_{2,k}$', '$e_{3,k}$', '$e_{4,k}$', '$e_{5,k}$', '$e_{6,k}$', '$e_{7,k}$',...
    'Interpreter', 'latex', 'location', 'east');
l.NumColumns = 1;

set(gca,'TickLabelInterpreter','latex')


subplot(5,1,2)
hold on
for i = 1:7
    plot(t_sim, q(i,:), '-', 'Color', jntColors{i})
    yline(q_UB(i), '--', 'Color', jntColors{i});
    yline(q_LB(i), '--', 'Color', jntColors{i});
end
hold off
grid on
xlabel('time (s)', 'Interpreter', 'latex')
ylabel('angle (rad)', 'Interpreter', 'latex'); 
title('Joint Trajectory $q_k$', 'Interpreter', 'latex')
set(gca,'TickLabelInterpreter','latex')

subplot(5,1,3)
hold on
for i = 1:7
    plot(t_sim, u(i,:), '-', 'Color', jntColors{i})
    yline(u_UB(i), '--', 'Color', jntColors{i});
    yline(u_LB(i), '--', 'Color', jntColors{i});
end
hold off
grid on
xlabel('time (s)', 'Interpreter', 'latex')
ylabel('angular rate (rad/s)', 'Interpreter', 'latex'); 
title('Control Input $u_k$', 'Interpreter', 'latex')
set(gca,'TickLabelInterpreter','latex')

subplot(5,1,4)
hold on
yyaxis left
plot(t, mpcTime);
ylabel('MPC time (s)', 'Interpreter', 'latex'); 
yyaxis right
plot(t, no_ds);
ylabel('num. obs. in downsampled p.c.', 'Interpreter', 'latex'); 
yline(dt)
% ylim([-0.05, 0.2])
hold off
grid on
xlabel('time (s)', 'Interpreter', 'latex')
title('MPC Computation Time and no. of obstacles', 'Interpreter', 'latex')
set(gca,'TickLabelInterpreter','latex')

subplot(5,1,5)
plot(t, md);
% plot(t, md_ds);
yline(r(1));
% legend('Minimum distance', 'interpreter', 'latex')
hold off
xlabel('time (s)', 'Interpreter', 'latex')
ylabel('minimum $\|d_{ij}(q_k)\|$ over $i,j$', 'Interpreter', 'latex'); 
title('Minimum obstacle distance', 'Interpreter', 'latex')
set(gca,'TickLabelInterpreter','latex')
grid on


f = gcf
exportgraphics(f,'figures/obs_MPC_x6_simtime.png','Resolution',500)

disp(['final error: ', num2str(err(:,end)')]);
disp(['error norm: ', num2str(norm(err(:,end)))]);

disp(['mean mpc time: ', num2str(mean(mpcTime))])





%%
pg = Tg(1:3, 4);
visualizeRobotTraj(robot, q, t_sim, pg, geom_xyz, 1)

%%

fig = figure; 
fig.Position = [200 200 1500 500];     % printing to figure
subplot(1,4,1)
hold on
plot3dpts(geom_xyz, 'r*')
robot.show(q0)
plot3(pg(1), pg(2), pg(3), 'g.', 'MarkerSize', 50);
view([-45 15]);

subplot(1,4,2)
hold on
plot3dpts(geom_xyz, 'r*')
robot.show(q(:,80))
plot3(pg(1), pg(2), pg(3), 'g.', 'MarkerSize', 50);
view([-45 15]);

subplot(1,4,3)
hold on
plot3dpts(geom_xyz, 'r*')
robot.show(q(:,130))
plot3(pg(1), pg(2), pg(3), 'g.', 'MarkerSize', 50);
view([-45 15]);

subplot(1,4,4)
hold on
plot3dpts(geom_xyz, 'r*')
robot.show(q(:,200))
plot3(pg(1), pg(2), pg(3), 'g.', 'MarkerSize', 50);
view([-45 15]);

f = gcf
exportgraphics(f,'figures/grasping_traj.png','Resolution',500)



