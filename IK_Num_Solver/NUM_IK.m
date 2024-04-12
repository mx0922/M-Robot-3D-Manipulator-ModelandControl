% IK numerical solver——逆运动学数值求解器
function Q_cal = NUM_IK(P_ref, Q0, model)

[~, ~, P_act] = FK(Q0, model);

tol = 1e-6;

Q_temp = Q0;

if norm(P_ref - P_act) <= tol
   Q_cal = Q_temp; 
end

while norm(P_ref - P_act) > tol
   J = Get_Jacobian_M(Q_temp, model); 
   delta_P = P_ref - P_act;
   Q_cal = Q_temp +  pinv(J) * delta_P;
   [~, ~, P_act] = FK(Q_cal, model);
   Q_temp = Q_cal;
end