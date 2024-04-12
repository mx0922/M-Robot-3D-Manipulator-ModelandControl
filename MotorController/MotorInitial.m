% �����ʼ��������һ�����б�Ҫ����Ȼ�������ֱ�Ӿͷɵ��ˣ�������ʹ�Ľ�ѵ����
function [Joint_act, motor1] = MotorInitial(Joint_ref, Joint_tau, motor)

Motor_ref = Joint_ref * motor.Gear_ratio;
Motor_tau = Joint_tau / motor.Gear_ratio;

motor.th = Motor_ref;
motor.tau = Motor_tau;

Joint_act(1) = motor.th / motor.Gear_ratio;
Joint_act(2) = motor.w / motor.Gear_ratio;
Joint_act(3) = motor.a / motor.Gear_ratio;

motor1 = motor;