clear;clc

%% Joint goal tracking
% Joint goal switching

N = 1:10;
final_error_norm = [
        0.00067694;
        2.8808e-06;
        1.5775e-07;
        4.0578e-08;
        2.2621e-08;
        1.7673e-08;
        1.5948e-08;
        1.5284e-08;
        1.5019e-08;
        1.4911e-08
    ];

mpc_time = [
        0.0018475;
        0.0024728;
        0.0019995;
        0.0022061;
        0.0022763;
        0.0026497;
        0.0027157;
        0.0039793;
        0.0040527;
        0.0035489
    ];

figure
subplot(2,1,1)
hold on
yyaxis left
plot(N, final_error_norm);
yyaxis right
plot(N, mpc_time);
title('Switching Goal Tracking', 'Interpreter', 'latex')
hold off
xlabel('MPC Horizon N', 'interpreter', 'latex')
legend('Final error: $\| q_N - q_g \|$ (rad)', 'Mean MPC time (s)', 'Interpreter', 'latex', 'location', 'best')
set(gca,'TickLabelInterpreter','latex')
grid on


% X6 tracking
% X6 goal switching

N = 1:10;

final_error_norm = [
        0.29271;
        0.25378;
        0.20695;
        0.20297;
        0.083316;
        0.022608;
        0.014013;
        0.012024;
        0.009217;
        0.0069005;
    ];

mpc_time = [
        0.0058844;
        0.0086664;
        0.011124;
        0.014615;
        0.018838;
        0.02821;
        0.039368;
        0.051248;
        0.057108;
        0.066235
    ];

subplot(2,1,2)
hold on
yyaxis left
plot(N, final_error_norm);
yyaxis right
plot(N, mpc_time);
title('Switching Pose Tracking', 'Interpreter', 'latex')
hold off
legend('Final error: $\| e_N \|$', 'Mean MPC time (s)', 'Interpreter', 'latex', 'location', 'best')
set(gca,'TickLabelInterpreter','latex')
xlabel('MPC Horizon N', 'interpreter', 'latex')
grid on

%%
f = gcf
exportgraphics(f,'figures/boxcon_horizon_performance.png','Resolution',500)

