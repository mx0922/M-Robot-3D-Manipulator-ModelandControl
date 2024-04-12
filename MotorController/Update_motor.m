% Update_motor Program
% 为了提高计算的速度，按照ode45的方式写了一个4阶Runge-Kutta微分方程求解器
h = motor.CL_T;
% SV —— State Vector
SV = [motor.th; motor.w; motor.I];

A = [0, 1, 0;
     0, 0, motor.Kt / motor.J;
     0, -motor.Ke / motor.L, -motor.R / motor.L];
B = [0; 0; 1 / motor.L];
C = [0; 1 / motor.J; 0];

k1 = h * (A * SV + B * motor.U + C * motor.tau);
k2 = h * (A * (SV + k1 / 2) + B * motor.U + C * motor.tau);
k3 = h * (A * (SV + k2 / 2) + B * motor.U + C * motor.tau);
k4 = h * (A * (SV + k3) + B * motor.U + C * motor.tau);

SV = SV + (k1 + 2 * k2 + 2 * k3 + k4) / 6;
dSV = A * SV + B * motor.U + C * motor.tau;

motor.th = SV(1);
motor.w = SV(2);
motor.I = SV(3);
motor.a = dSV(2);