function calibrateMotor(device, ODriveEnums)
% ODrive Motor Calibration for D6374 150KV
% https://docs.odriverobotics.com/v/latest/guides/odrivetool-setup.html

%% Local Variables
% Voltage/Current Trip Levels
overvoltageTripLevel    = 17;
max_positive_current    = 2.24;
max_negative_current    = -0.01;

% Brake resistor
% resistorEnable          = 
resistorResistance      = 2.1;

% Motor Params
motorType               = ODriveEnums.MotorType.MOTOR_TYPE_HIGH_CURRENT;
torqueConstant          = 8.27/150;
polePairs               = 7;
current_softMax         = 70;
current_hardMax         = 90;
calibrationCurrent      = 10;
resCalibMaxVoltage      = 2;
calib_lockingCurrent    = 10;
motorInputMode          = ODriveEnums.InputMode.INPUT_MODE_PASSTHROUGH;
motorControlMode        = ODriveEnums.ControlMode.CONTROL_MODE_TORQUE_CONTROL;
motorVel_limit          = 3;
motorVel_limitTolerance = 1.66666666;

% Thermistor Calibration (Params from datasheet: https://www.sensit.cz/Download.aspx?param=jx3godhnp7M1LWTdfUhReAYjUz0y87G8D6DymM3Ud9qluJ9AdBgcB3ZaPGxMsYURzuYhZRSIFcuoTPJPrWrOIyvn%2BBf7hGFG1L0eQOvRR1zens%2BNaYOv5%2Bn%2FDDOItlUCv9g99zwDyQGtO4w4HnmZE3rhanduj%2F0RBJSbJR9k%2FDbDFisW999ZG7CJHe%2BOwvrDfqfEfkvlr7fyY%2F50YpwuFeugNMXy1GfSZ3Ojyou5DploLP36hJfVowdUK4AnMk%2BVk1srYw2iRGdGVM80mUK9fCqMktayatmACdzGg3p3VaHGWmOO938GHHGxk6KF1aIdp6eUte2aRdade9We49FypYwyLPd%2BGixsPoJmO7rBgUAPGvwubyLNyaB0l3RmTYVY7LSNDGR0uYVez%2Ffh4eY3Jo4tzCznykvrQf457T3GYBFKfy4VvOWk5zmh2TkwWQBJpOPBUCl4biycwNqelfltUXPomYxfj6PR1uDqSPr7NLRALtHH2Ls9Ly1DGJuuDWTKWLO8IIdzX0e60WwNZN%2FWVg%3D%3D&tname=SiteContent.aspx )
t_ref       = 25;                                                                           % Reference temperature [Deg.Celsius]
t_min       = -40;                                                                          % Min temperature [Deg.Celsius]
t_max       = 125;                                                                          % Max temperature [Deg.Celsius]
R_25        = 10*10^3;                                                                      % Resistance at t_ref
B_25_85     = 3435;                                                                         % Beta value...

% Communication
canConfigProtocol       = ODriveEnums.Protocol.PROTOCOL_NONE;
% enableUart              = True
uartBaudrate            = 115200;

% Encoder 
encoderConfigMode       = ODriveEnums.Rs485EncoderMode.RS485_ENCODER_MODE_AMT21_EVENT_DRIVEN;
loadEncoder             = ODriveEnums.EncoderId.ENCODER_ID_RS485_ENCODER0;
commutationEncoder       = ODriveEnums.EncoderId.ENCODER_ID_RS485_ENCODER0;



%% Write Parameters to ODrive

% Voltage/Current Trip Levels
writeParameter(device, "config.dc_bus_overvoltage_trip_level", overvoltageTripLevel)
writeParameter(device, "config.dc_max_positive_current", max_positive_current)
writeParameter(device, "config.dc_max_negative_current", max_negative_current)

% Brake resistor
writeParameter(device, "config.brake_resistor0.enable", True)
writeParameter(device, "config.brake_resistor0.resistance", resistorResistance)
writeline(device, "sc") % Clear errors ?=?????

% Motor Parameters
writeParameter(device, "axis0.config.motor.motor_type", motorType)
writeParameter(device, "axis0.config.motor.pole_pairs", polePairs)
writeParameter(device, "axis0.config.motor.torque_constant", torqueConstant)

writeParameter(device, "axis0.config.motor.current_soft_max", current_softMax)
writeParameter(device, "axis0.config.motor.current_hard_max", current_hardMax)
writeParameter(device, "axis0.config.motor.calibration_current", calibrationCurrent)
writeParameter(device, "axis0.config.motor.resistance_calib_max_voltage", resCalibMaxVoltage)
writeParameter(device, "axis0.config.calibration_lockin.current", calib_lockingCurrent)
writeParameter(device, "axis0.controller.config.input_mode", motorInputMode)
writeParameter(device, "axis0.controller.config.control_mode", motorControlMode)

writeParameter(device, "axis0.controller.config.vel_limit", motorVel_limit)
writeParameter(device, "axis0.controller.config.vel_limit_tolerance", motorVel_limitTolerance)

% Thermistor
writeParameter(device, "axis0.motor.motor_thermistor.config.r_ref", R_25)
writeParameter(device, "axis0.motor.motor_thermistor.config.beta_ref", B_25_85)
writeParameter(device, "axis0.motor.motor_thermistor.config.t_ref", t_ref)
writeParameter(device, "axis0.motor.motor_thermistor.config.temp_limit_lower", t_min)
writeParameter(device, "axis0.motor.motor_thermistor.config.temp_limit_upper", t_max)
writeParameter(device, "axis0.motor.motor_thermistor.config.enabled", True)


% Communication
writeParameter(device, "can.config.protocol", canConfigProtocol)
writeParameter(device, "config.enable_uart_a", True)
writeParameter(device, "config.uart_a_baudrate", uartBaudrate)

% Encoder
writeParameter(device, "rs485_encoder_group0.config.mode", encoderConfigMode)
writeParameter(device, "axis0.config.load_encoder", loadEncoder)
writeParameter(device, "odrv.axis0.config.commutation_encoder", commutationEncoder)

%% Motor Calibration


writeParameter(device, "axis0.requested_state", ODriveEnums.AxisState.AXIS_STATE_MOTOR_CALIBRATION) % Calibration state
% [wait for end of motor beep] , HOW THE FUCK
pause(10)




