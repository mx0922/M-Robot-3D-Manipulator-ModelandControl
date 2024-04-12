function J = Get_Jacobian_M(Q, model)

q1 = Q(1);    q2 = Q(2);   q3 = Q(3);
L1 = model.L1;   L2 = model.L2;    L3 = model.L3;

J = [ - L3*(cos(q1)*cos(q3) + sin(q1)*sin(q2)*sin(q3)) - L2*sin(q1)*sin(q2), cos(q1)*cos(q2)*(L2 + L3*sin(q3)),  L3*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2));
        L2*cos(q1)*sin(q2) - L3*(cos(q3)*sin(q1) - cos(q1)*sin(q2)*sin(q3)), cos(q2)*sin(q1)*(L2 + L3*sin(q3)), L3*cos(q3)*sin(q1)*sin(q2) - L3*cos(q1)*sin(q3);
                                                                          0,        -sin(q2)*(L2 + L3*sin(q3)),                              L3*cos(q2)*cos(q3)];
