function Joint_tau_ctrl = JointController(i, Q_ref_r, Q_act_r, control_T)

q1_ref = Q_ref_r(i, 1);     q2_ref = Q_ref_r(i, 2);     q3_ref = Q_ref_r(i, 3);
q1_act = Q_act_r(i, 1);     q2_act = Q_act_r(i, 2);     q3_act = Q_act_r(i, 3);

Kp = 10;   Kd = 1;

% 参考角速度ref
if i == 1
    dq1_ref = 0;
    dq2_ref = 0;
    dq3_ref = 0;
else
    dq1_ref = (Q_ref_r(i, 1) - Q_ref_r(i - 1, 1)) / control_T;
    dq2_ref = (Q_ref_r(i, 2) - Q_ref_r(i - 1, 2)) / control_T;
    dq3_ref = (Q_ref_r(i, 3) - Q_ref_r(i - 1, 3)) / control_T;
end

% 实际角速度act
if i == 1 || i == 2
    dq1_act = 0;
    dq2_act = 0;
    dq3_act = 0;
else
    dq1_act = (Q_act_r(i - 1, 1) - Q_act_r(i - 2, 1)) / control_T;
    dq2_act = (Q_act_r(i - 1, 2) - Q_act_r(i - 2, 2)) / control_T;
    dq3_act = (Q_act_r(i - 1, 3) - Q_act_r(i - 2, 3)) / control_T;
end

dq_err = [dq1_ref; dq2_ref; dq3_ref] - [dq1_act; dq2_act; dq3_act];
q_err = [q1_ref; q2_ref; q3_ref] - [q1_act; q2_act; q3_act];

if i == 1
    Joint_tau_ctrl = [0; 0; 0];
else
    Joint_tau_ctrl = Kp * q_err + Kd * dq_err;
end

Tau_lim = 20.0;
for i = 1:3
   if abs(Joint_tau_ctrl(i)) > Tau_lim
       Joint_tau_ctrl(i) = sign(Joint_tau_ctrl(i)) * Tau_lim;
   end
end