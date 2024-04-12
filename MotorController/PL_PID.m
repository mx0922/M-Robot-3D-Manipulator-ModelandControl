% PL_PID
P_err = motor.th_ref - motor.th;
motor.PL_sumerr  = motor.PL_sumerr + P_err * motor.CL_T;
motor.PL_derr = (P_err - motor.PL_err) / motor.CL_T;
motor.PL_err = P_err;
motor.w_ref = motor.PL_P * motor.PL_err + motor.PL_I * motor.PL_sumerr + motor.PL_D * motor.PL_derr;

if abs(motor.w_ref) > motor.w_lim
    motor.w_ref = sign(motor.w_ref) * motor.w_lim;
end