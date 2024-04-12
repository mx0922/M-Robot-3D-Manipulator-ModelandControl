% CL_PID
C_err = motor.I_ref - motor.I;
motor.CL_sumerr  = motor.CL_sumerr + C_err * motor.CL_T;
motor.CL_derr = (C_err - motor.CL_err) / motor.CL_T;
motor.CL_err = C_err;
motor.U = motor.CL_P * motor.CL_err + motor.CL_I * motor.CL_sumerr + motor.CL_D * motor.CL_derr;

if abs(motor.U) > motor.U_lim
    motor.U = sign(motor.U) * motor.U_lim;
end