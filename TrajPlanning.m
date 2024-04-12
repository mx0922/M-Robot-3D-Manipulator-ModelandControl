%% trajectory planning
% P0����>(-0.3, -0.4, 0.3)����>д��һ��M��������>�ص�P0
% Output: P3_r ���� �����˶�����ĩ�����꣨Cartisian Coordinate��

%------------------------------NOTE-------------------------------------------
% �켣�滮�Ĺ�������Ҫ�ر�ע�����·��棨�����׵������˶�ѧ�󲻳��⣩��
% 1. �������ù滮��·��������е�۵Ļ��Χ����MRobotû�ӹؽ���λ����˻��Χ�ȽϿ���
% 2. ����Ҫ��һ�㣺�滮��·��������Ҫ�����ſ˱��������������ʹIK�������������ѭ��
% 3. ���Թ켣�滮�����Ǽ�򵥵����û������˶����Ķ����˶����Ķ������⣬�տ�ʼ�Ҿ���
%    ̫������������°�����ⲻ�����˶�ѧ������

% ���˶�ѧ���ĩ�˵ĳ�ʼλ��
[~, ~, P3_0] = FK(z0, model); 

P3_x_r = zeros(3001, 1);
P3_y_r = P3_x_r;    P3_z_r = P3_x_r;

%% Step 1: ��P3_0 �˶��� P3_init(-0.4; 0.7; 0.5)
P3_init = [-0.4; 0.7; 0.5];
T_init = 2.0;
N_init = T_init / control_T;
for i = 1:N_init + 1
   t_temp = (i - 1) * control_T;
   %-------------------TSpline(p0,v0,t0,p1,t1,p2,v2,t2,t)
   [P3_x_r(i), ~, ~] = TSpline(P3_0(1), 0, 0, (P3_0(1) + P3_init(1)) / 2, T_init / 2, P3_init(1), 0, T_init, t_temp);
   [P3_y_r(i), ~, ~] = TSpline(P3_0(2), 0, 0, (P3_0(2) + P3_init(2)) / 2, T_init / 2, P3_init(2), 0, T_init, t_temp);
   [P3_z_r(i), ~, ~] = TSpline(P3_0(3), 0, 0, (P3_0(3) + P3_init(3)) / 2, T_init / 2, P3_init(3), 0, T_init, t_temp);
end

%% Step 2: ��P3_init �˶��� 'M' ����
T_run = 8.0;
N_run = T_run / control_T;
for i = 1:N_run
   t = i * control_T;
   k = i + N_init + 1;
   P3_y_r(k) = P3_init(2);
   if t <= 2.0
       t_temp1 = t;
       P3_x_r(k) = -0.4 + 0.2 * t_temp1 / 2;
       P3_z_r(k) = 0.5 + 0.3 * t_temp1 / 2;
   elseif t > 2.0 && t <= 4.0
       t_temp2 = t - 2.0;
       P3_x_r(k) = -0.2 + 0.2 * t_temp2 / 2;
       P3_z_r(k) = 0.8 - 0.3 * t_temp2 / 2;
   elseif t > 4.0 && t <= 6.0
       t_temp3 = t - 4.0;
       P3_x_r(k) = 0 + 0.2 * t_temp3 / 2;
       P3_z_r(k) = 0.5 + 0.3 * t_temp3 / 2;
   else
       t_temp4 = t - 6.0;
       P3_x_r(k) = 0.2 + 0.2 * t_temp4 / 2;
       P3_z_r(k) = 0.8 - 0.3 * t_temp4 / 2;
   end
end

%% Step 3: д��M�����ص���ʼλ��P3_0
T_reset = 2.0;
P3_end = [0.4; 0.7; 0.5];
N_reset = T_reset / control_T;
for i = 1:N_reset
    k = i + N_init + 1 + N_run;
    t_temp = i * control_T;
   %-------------------TSpline(p0,v0,t0,p1,t1,p2,v2,t2,t)
   [P3_x_r(k), ~, ~] = TSpline(P3_end(1), 0, 0, (P3_0(1) + P3_end(1)) / 2, T_reset / 2, P3_0(1), 0, T_reset, t_temp);
   [P3_y_r(k), ~, ~] = TSpline(P3_end(2), 0, 0, (P3_0(2) + P3_end(2)) / 2, T_reset / 2, P3_0(2), 0, T_reset, t_temp);
   [P3_z_r(k), ~, ~] = TSpline(P3_end(3), 0, 0, (P3_0(3) + P3_end(3)) / 2, T_reset / 2, P3_0(3), 0, T_reset, t_temp);
end
% �����˶�����ĩ�˹켣
P3_r = [P3_x_r, P3_y_r, P3_z_r];