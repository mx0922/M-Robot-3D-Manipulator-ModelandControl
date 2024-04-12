% SL_PID
S_err = motor.w_ref - motor.w;
motor.SL_sumerr  = motor.SL_sumerr + S_err * motor.CL_T;
motor.SL_derr = (S_err - motor.SL_err) / motor.CL_T;
motor.SL_err = S_err;
motor.I_ref = motor.SL_P * motor.SL_err + motor.SL_I * motor.SL_sumerr + motor.SL_D * motor.SL_derr;

if abs(motor.I_ref) > motor.I_lim
    motor.I_ref = sign(motor.I_ref) * motor.I_lim;
end