% Forward Kinematics(IK)——正运动学
function [P1, P2, P3] = FK(z, model)

q1 = z(1);    q2 = z(2);   q3 = z(3);
L1 = model.L1;   L2 = model.L2;    L3 = model.L3;

P1 = [0; 0; L1];
P2 = [L2*cos(q1)*sin(q2);
      L2*sin(q1)*sin(q2);
        L1 + L2*cos(q2)];
P3 = [L2*cos(q1)*sin(q2) - L3*(cos(q3)*sin(q1) - cos(q1)*sin(q2)*sin(q3));
      L3*(cos(q1)*cos(q3) + sin(q1)*sin(q2)*sin(q3)) + L2*sin(q1)*sin(q2);
                                    L1 + L2*cos(q2) + L3*cos(q2)*sin(q3)];