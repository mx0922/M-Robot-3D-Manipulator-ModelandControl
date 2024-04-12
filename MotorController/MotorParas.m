% global motor
motor.Gear_ratio = 30;

motor.U_lim = 24;
motor.I_lim = 10;
motor.w_lim = 10;

motor.CL_T = 1/10000;
motor.SL_T = 1/5000;
motor.PL_T = 1/1000;

motor.L = 7.667E-5;
motor.R = 2.692E-2;
motor.J = 1.15E-4;
motor.Kt = 6.83E-2;
motor.Ke = 1E-5;

motor.CL_P = 0.0340402631890567;      motor.CL_I = 22.1103198771869;      motor.CL_D = 0;
motor.SL_P = 0.99555938075245;      motor.SL_I = 91.3046132799886;      motor.SL_D = 0.00140955886990918;
motor.PL_P = 1149.5816231966;      motor.PL_I = 130531.418192734;      motor.PL_D = 2.02045034547429;

motor.U = 0;
motor.I = 0;    motor.I_ref = 0;
motor.w = 0;    motor.w_ref = 0;
motor.th = 0;   motor.th_ref = 0;
motor.a = 0;

motor.CL_err = 0;   motor.CL_sumerr = 0;    motor.CL_derr = 0;
motor.SL_err = 0;   motor.SL_sumerr = 0;    motor.SL_derr = 0;
motor.PL_err = 0;   motor.PL_sumerr = 0;    motor.PL_derr = 0;

motor.tau = 0;

motor.SL_flag = 0;
motor.PL_flag = 0;