function calibrateMotor(device)
% ODrive Motor Calibration for D6374 150KV
% https://docs.odriverobotics.com/v/latest/guides/odrivetool-setup.html


% Motor Calibration
writeline(device, "w axis0.config.motor.motor_type " + string(ODriveEnums.MotorType.MOTOR_TYPE_HIGH_CURRENT))
writeline(device, "w axis0.config.motor.pole_pairs 7")
writeline(device, "w axis0.config.motor.torque_constant " + string(8.27/150))
writeline(device, "w axis0.requested_state " + string(ODriveEnums.AxisState.AXIS_STATE_MOTOR_CALIBRATION))
% [wait for end of motor beep] , HOW THE FUCK
pause(10)

writeline(device, "w save_configuration()")

% % Thermistor Calibration (Params from datasheet: https://www.sensit.cz/Download.aspx?param=jx3godhnp7M1LWTdfUhReAYjUz0y87G8D6DymM3Ud9qluJ9AdBgcB3ZaPGxMsYURzuYhZRSIFcuoTPJPrWrOIyvn%2BBf7hGFG1L0eQOvRR1zens%2BNaYOv5%2Bn%2FDDOItlUCv9g99zwDyQGtO4w4HnmZE3rhanduj%2F0RBJSbJR9k%2FDbDFisW999ZG7CJHe%2BOwvrDfqfEfkvlr7fyY%2F50YpwuFeugNMXy1GfSZ3Ojyou5DploLP36hJfVowdUK4AnMk%2BVk1srYw2iRGdGVM80mUK9fCqMktayatmACdzGg3p3VaHGWmOO938GHHGxk6KF1aIdp6eUte2aRdade9We49FypYwyLPd%2BGixsPoJmO7rBgUAPGvwubyLNyaB0l3RmTYVY7LSNDGR0uYVez%2Ffh4eY3Jo4tzCznykvrQf457T3GYBFKfy4VvOWk5zmh2TkwWQBJpOPBUCl4biycwNqelfltUXPomYxfj6PR1uDqSPr7NLRALtHH2Ls9Ly1DGJuuDWTKWLO8IIdzX0e60WwNZN%2FWVg%3D%3D&tname=SiteContent.aspx )
t_ref       = 25;       % Reference temperature [Deg.Celsius]
t_min       = -40;      % Min temperature [Deg.Celsius]
t_max       = 125;      % Max temperature [Deg.Celsius]
R_25        = 10*10^3;  % Resistance at t_ref
B_25_85     = 3435;     % Beta value...

writeline(device, "w axis0.motor.motor_thermistor.config.r_ref " + string(R_25))
writeline(device, "w axis0.motor.motor_thermistor.config.beta_ref " + string(B_25_85))
writeline(device, "w axis0.motor.motor_thermistor.config.t_ref " + string(t_ref))
writeline(device, "w axis0.motor.motor_thermistor.config.temp_limit_lower " + string(t_min)) 
writeline(device, "w axis0.motor.motor_thermistor.config.temp_limit_upper " + string(t_max)) 
writeline(device, "w axis0.motor.motor_thermistor.config.enabled " + string(true)) 

