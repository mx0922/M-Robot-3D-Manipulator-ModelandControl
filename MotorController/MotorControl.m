% Motor Control Program 电机控制程序 
function [Joint_act, motor1] = MotorControl(Joint_ref, Joint_tau, motor)

% joint ——> motor
Motor_ref = Joint_ref * motor.Gear_ratio;
Motor_tau = Joint_tau / motor.Gear_ratio;

motor.th_ref = Motor_ref;
motor.tau = Motor_tau;

for i = 1:motor.PL_T / motor.CL_T
    PL_PID;
    SL_PID;
    CL_PID;
    Update_motor;
end

% motor ——> joint
Joint_act(1) = motor.th / motor.Gear_ratio;
Joint_act(2) = motor.w / motor.Gear_ratio;
Joint_act(3) = motor.a / motor.Gear_ratio;

motor1 = motor;
