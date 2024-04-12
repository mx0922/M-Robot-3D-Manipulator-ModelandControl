%----------------------  MRobot  --------------------------
% Introduction��Three degrees of freedoms space manipulator.
% Target Motion��Write an 'M' character in three dimensions.
% Author��Xiang Meng ����     E-mail: mengxiang@bit.edu.cn
% Date��2020.7.4
%----------------  Copyright Reserved  --------------------
clear
close all
clc

addpath .\IK_Num_Solver
addpath .\MotorController

%% Part 1: ��е��ģ�����������趨
% 1. �����ɶȻ�е��ģ�Ͳ���
ModelParas;

% 2. �����������8526���
MotorParas;
% �������ɶȶ�Ӧ�ĵ��
motor1 = motor;     motor2 = motor;     motor3 = motor;

% 3. ��������
control_T = 0.004;

% 4. ��ʼ״̬
q1 = pi/3;  q2 = pi/6;  q3 = pi/3;
dq1 = 0; dq2 = 0; dq3 = 0;
z0 = [q1 q2 q3 dq1 dq2 dq3]';

%% Part 2: �켣�滮 + ��ֵ������˶�ѧ
% 1. �켣�滮�����õ�ĩ�˵�����������ϵ�е�λ�á���P3_r
TrajPlanning;

% 2. ��ֵ������˶�ѧ
prog_N = length(P3_r);
% �ο��ؽڽǶȼ�¼
Q_ref_r = zeros(prog_N, 3);

for i = 1:prog_N
    if i == 1
        Q0 = z0(1:3);
    else
        Q0 = Q_ref_r(i - 1, :)';
    end
    Q_ref = NUM_IK(P3_r(i, :)', Q0, model);
    Q_ref_r(i, :) = Q_ref';
end

%% Part 3: λ�ò�֡���>���ٶ�dQ���ٶȲ�֡���>�Ǽ��ٶ�ddQ ����> �涯��ѧ��ID����⸺��ת��TAU
% 1. �������
dQ_ref_r = zeros(prog_N, 3);        ddQ_ref_r = zeros(prog_N, 3);
for i = 1:prog_N
    if i > 1
        dQ_ref_r(i, :) = (Q_ref_r(i, :) - Q_ref_r(i - 1, :)) / control_T;
    end
    if i > 2
        ddQ_ref_r(i, :) = (dQ_ref_r(i, :) - dQ_ref_r(i - 1, :)) / control_T;
    end
end

% 2. ID��⸺��ת��TAU
Joint_tau = zeros(prog_N, 3);
TAU_lim = 50.0;
for i = 1:prog_N
    TAU = ID_MX(i, Q_ref_r, dQ_ref_r, ddQ_ref_r, model);
    % ��������
    for j = 1:3
        if abs(TAU(j)) > TAU_lim
            TAU(j) = sign(TAU(j)) * TAU_lim;
        end
    end
    Joint_tau(i, :) = TAU';
end

%% Part 4: ������� Motor Controller ����> �ؽڵ�ʵ�ʽǶȣ�Q_act_r
Q_act_r = zeros(prog_N, 3);
Joint_tau_ctrl_r = zeros(prog_N, 3);
h = waitbar(0, 'Please wait...');
s = clock;

for i = 1:prog_N
    k = control_T / motor.PL_T;
    Joint_ref1 = Q_ref_r(i, 1);     Joint_ref2 = Q_ref_r(i, 2);     Joint_ref3 = Q_ref_r(i, 3);
    
    Joint_tau_ctrl = JointController(i, Q_ref_r, Q_act_r, control_T);
    Joint_tau_ctrl_r(i, :) = Joint_tau_ctrl';
    
    Joint_tau1 = Joint_tau(i, 1) + Joint_tau_ctrl(1);   Joint_tau2 = Joint_tau(i, 2) + Joint_tau_ctrl(2);   Joint_tau3 = Joint_tau(i, 3) + Joint_tau_ctrl(3);
    if i == 1
        % �����ʼ��(���б�Ҫ������)
        [Joint_act1, motor1] = MotorInitial(Joint_ref1, Joint_tau1, motor1);
        [Joint_act2, motor2] = MotorInitial(Joint_ref2, Joint_tau2, motor2);
        [Joint_act3, motor3] = MotorInitial(Joint_ref3, Joint_tau3, motor3);
    else
        for j = 1:k
           [Joint_act1, motor1] = MotorControl(Joint_ref1, Joint_tau1, motor1);
           [Joint_act2, motor2] = MotorControl(Joint_ref2, Joint_tau2, motor2);
           [Joint_act3, motor3] = MotorControl(Joint_ref3, Joint_tau3, motor3);
        end
    end
    Q_act_r(i, :) = [Joint_act1(1), Joint_act2(1), Joint_act3(1)];
    
    % ����ʱ����ʾbar
    if i == 10
        is = etime(clock, s);
        esttime = is / 10 * prog_N;
    elseif i > 10
        h = waitbar(i / prog_N, h, ...
            ['Remaining Time = ', num2str(esttime - etime(clock, s), '%4.1f'), 'sec']);
    end
end
close(h);

%% Part 5: Output Plot ���չʾ
% 1.�ؽڶ˲ο��Ƕ���ʵ�ʽǶȵ�����
figure(1);
t = 0:control_T:T_init + T_run + T_reset;
plot(t, Q_ref_r(:, 1), t, Q_act_r(:, 1));
hold on
plot(t, Q_ref_r(:, 2), t, Q_act_r(:, 2));
plot(t, Q_ref_r(:, 3), t, Q_act_r(:, 3));
xlabel('Time [s]');
ylabel('Joint Angle [rad]');
title('Angle-Ref and Angle-Act of Three Joints');
legend({'q1-ref', 'q1-act', 'q2-ref', 'q2-act', 'q3-ref', 'q3-act'}, 'FontSize', 16);

figure(2);
plot(t, Q_ref_r(:, 1) - Q_act_r(:, 1));
hold on
plot(t, Q_ref_r(:, 2) - Q_act_r(:, 2));
plot(t, Q_ref_r(:, 3) - Q_act_r(:, 3));
xlabel('Time [s]');
ylabel('Joint Angle Error [rad]');
title('Joint Angle Error of Three Joints');
legend({'q1-error', 'q2-error', 'q3-error'}, 'FontSize', 12);

figure(3);
j = 1;
num = 10;
P3_plot = [];
for i = 1:num:prog_N
   pause(0.001);
   [P1, P2, P3] = FK(Q_act_r(i, :), model);   
   MRobotShowTime;
   F(j) = getframe(gcf);
   j = j + 1;
end

v = VideoWriter('MRobot_Drawing.avi');
v.Quality = 90;
open(v);
writeVideo(v,F);
close(v);