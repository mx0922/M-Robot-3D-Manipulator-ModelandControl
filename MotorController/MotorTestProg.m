% motor test program
clear
clc

% global motor
MotorParas;
motor1 = motor;
m = 10;  l = 0.5;  g = 9.81;  

control_T = 0.004;

t = 0:control_T:2; 
PROG_N = length(t);

th = zeros(PROG_N, 1);
w = th;     a = th;

Joint_act_r = zeros(PROG_N, 3);

for i = 1:PROG_N
   t_temp = (i - 1) * control_T;
   [th(i), w(i), a(i)] = TSpline(0, 0, 0, pi/6, 1.0, pi/3, 0, 2.0, t_temp);
end

Joint_tau = zeros(PROG_N, 1);
for i = 1:PROG_N
    Joint_tau(i) = m * l * l * a(i) + m * g * l * sin(th(i));
    Joint_ref = th(i);
    k = control_T / motor.PL_T;
    for j = 1:k
        [Joint_act, motor1] = MotorControl(Joint_ref, Joint_tau(i), motor1);
    end
    Joint_act_r(i, :) = Joint_act;
end

figure(1);
plot(t, Joint_act_r(:, 1), 'r-', t, th, 'g-.','LineWidth', 2);