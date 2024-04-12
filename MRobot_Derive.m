clear
close all
clc

syms L1 L2 L3 M1 M2 M3 g T1 T2 T3 I1_1 I1_2 I1_3 I2_1 I2_2 I2_3 I3_1 I3_2 I3_3 real
syms q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 real

I1 = diag([I1_1 I1_2 I1_3]);    I2 = diag([I2_1 I2_2 I2_3]);    I3 = diag([I3_1 I3_2 I3_3]);

q = [q1 q2 q3]';    dq = [dq1 dq2 dq3]';    ddq = [ddq1 ddq2 ddq3]';

% ------------参考坐标系单位方向向量--------------
i = [1 0 0]';   j = [0 1 0]';   k = [0 0 1]';

Rzq1 = [cos(q1) * i + sin(q1) * j, -sin(q1) * i + cos(q1) * j, k];
Ryq2 = [cos(q2) * i - sin(q2) * k, j, sin(q2) * i + cos(q2) * k];
Rxq3 = [i, cos(q3) * j + sin(q3) * k, -sin(q3) * j + cos(q3) * k];

% R1 = Rot('z', q1);      R2 = Rot('y', q2);      R3 = Rot('x', q3); % 检查一下旋转矩阵正确性

R1 = Rzq1;  R12 = Rzq1 * Ryq2;    R123 = Rzq1 * Ryq2 * Rxq3;

% 各连杆惯量在——世界坐标系（world）中——的表示
I1_w = R1*I1*R1';   I2_w = R12*I2*R12';     I3_w = R123*I3*R123';

% 旋转后的各连杆坐标系的坐标轴单位向量在世界坐标系中的表示
X1 = R1(:,1);   Y1 = R1(:,2);   Z1 = R1(:,3);
X2 = R12(:,1);  Y2 = R12(:,2);  Z2 = R12(:,3);
X3 = R123(:,1); Y3 = R123(:,2); Z3 = R123(:,3);

% 各连杆质心以及末端在世界坐标系中的表示——随着初始姿态的改变而改变
r_O_G1 = L1 / 2 * Z1;   r_O_E1 = L1 * Z1;
r_E1_G2 = L2 / 2 * Z2;  r_E1_E2 = L2 * Z2;
r_O_G2 = r_O_E1 + r_E1_G2;  r_O_E2 = r_O_E1 + r_E1_E2;
r_E2_G3 = L3 / 2 * Y3;  r_E2_E3 = L3 * Y3;
r_O_G3 = r_O_E2 + r_E2_G3;  r_O_E3 = r_O_E2 + r_E2_E3;

J_mx = jacobian(r_O_E3, q);

% 角速度——相对于世界坐标系（world）
om1 = dq1 * Z1;
om2 = om1 + dq2 * Y2;
om3 = om2 + dq3 * X3;

% 各连杆质心速度
v_O = 0;
v_G1 = v_O + cross(om1,r_O_G1);
v_E1 = v_O + cross(om1,r_O_E1);

v_G2 = v_E1 + cross(om2,r_E1_G2);
v_E2 = v_E1 + cross(om2,r_E1_E2);

v_G3 = v_E2 + cross(om3,r_E2_G3);

% Lagrangian Equations
KE = 1/2 * M1 * dot(v_G1, v_G1) + 1/2 * M2 * dot(v_G2, v_G2) + 1/2 * M3 * dot(v_G3, v_G3)...
    + 1/2 * om1' * I1_w * om1 + 1/2 * om2' * I2_w * om2 + 1/2 * om3' * I3_w * om3;
PE = M1 * g * dot(r_O_G1, k) + M2 * g * dot(r_O_G2, k) + M3 * g * dot(r_O_G3, k);

L = KE - PE;
dL_dq = jacobian(L, dq);
ddL_dq_dt = simplify(jacobian(dL_dq, q) * dq + jacobian(dL_dq, dq) * ddq);
dL_q = jacobian(L, q)';

L1 = ddL_dq_dt(1) - dL_q(1) - T1;
L2 = ddL_dq_dt(2) - dL_q(2) - T2;
L3 = ddL_dq_dt(3) - dL_q(3) - T3;

eqn1 = collect(simplify(L1), [ddq1 ddq2 ddq3]);
eqn2 = collect(simplify(L2), [ddq1 ddq2 ddq3]);
eqn3 = collect(simplify(L3), [ddq1 ddq2 ddq3]);

RHS1 = -subs(eqn1, [ddq1 ddq2 ddq3], [0 0 0]);
M11 = subs(eqn1, [ddq1 ddq2 ddq3], [1 0 0]) + RHS1;
M12 = subs(eqn1, [ddq1 ddq2 ddq3], [0 1 0]) + RHS1;
M13 = subs(eqn1, [ddq1 ddq2 ddq3], [0 0 1]) + RHS1;

RHS2 = -subs(eqn2, [ddq1 ddq2 ddq3], [0 0 0]);
M21 = subs(eqn2, [ddq1 ddq2 ddq3], [1 0 0]) + RHS2;
M22 = subs(eqn2, [ddq1 ddq2 ddq3], [0 1 0]) + RHS2;
M23 = subs(eqn2, [ddq1 ddq2 ddq3], [0 0 1]) + RHS2;

RHS3 = -subs(eqn3, [ddq1 ddq2 ddq3], [0 0 0]);
M31 = subs(eqn3, [ddq1 ddq2 ddq3], [1 0 0]) + RHS3;
M32 = subs(eqn3, [ddq1 ddq2 ddq3], [0 1 0]) + RHS3;
M33 = subs(eqn3, [ddq1 ddq2 ddq3], [0 0 1]) + RHS3;

M = [M11 M12 M13; M21 M22 M23; M31 M32 M33];
RHS = [RHS1; RHS2; RHS3];
