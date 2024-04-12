%% trajectory planning
% P0——>(-0.3, -0.4, 0.3)——>写出一个M字样——>回到P0
% Output: P3_r —— 整个运动过程末端坐标（Cartisian Coordinate）

%------------------------------NOTE-------------------------------------------
% 轨迹规划的过程中需要特别注意以下方面（很容易导致逆运动学求不出解）：
% 1. 不可以让规划的路径超出机械臂的活动范围！（MRobot没加关节限位，因此活动范围比较宽泛）
% 2. 很重要的一点：规划的路径尽量不要出现雅克比奇异的情况，这会使IK求解器进入无限循环
% 3. 所以轨迹规划并不是简简单单想让机器人运动到哪儿就运动到哪儿的问题，刚开始我就是
%    太单纯，结果导致半天求解不出逆运动学！！！

% 正运动学求解末端的初始位置
[~, ~, P3_0] = FK(z0, model); 

P3_x_r = zeros(3001, 1);
P3_y_r = P3_x_r;    P3_z_r = P3_x_r;

%% Step 1: 从P3_0 运动到 P3_init(-0.4; 0.7; 0.5)
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

%% Step 2: 从P3_init 运动出 'M' 字样
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

%% Step 3: 写完M字样回到初始位置P3_0
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
% 整个运动过程末端轨迹
P3_r = [P3_x_r, P3_y_r, P3_z_r];