% 三自由度机械臂模型参数
model.L1 = 0.5;   model.L2 = 0.5;    model.L3 = 0.5;
model.M1 = 5.0;   model.M2 = 5.0;    model.M3 = 5.0;
model.g = 9.81;
model.I1 = diag([0.1073 0.1073 0.00625]);     model.I2 = model.I1;    model.I3 = model.I1;