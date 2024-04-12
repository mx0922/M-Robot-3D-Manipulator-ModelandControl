clear
close all
clc

load('.\Outputs\err_without_ctrl.mat');
load('.\Outputs\err_with_ctrl.mat');

prog_N = length(err_with_ctrl);
control_T = 0.004;

t = 0:control_T:(prog_N - 1) * control_T;

figure;
plot(t, err_without_ctrl(:, 1), 'r-', t, err_with_ctrl(:, 1), 'g-.');
hold on
plot(t, err_without_ctrl(:, 1) - err_with_ctrl(:, 1), 'b--');
xlabel('Time [s]');
ylabel('Joint Angle Error');