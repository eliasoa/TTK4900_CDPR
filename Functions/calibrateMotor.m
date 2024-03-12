function calibrateMotor(device, ODriveEnums)

% Erase existing config and reboot
eraseConfig(device)
pause(1)

% Add new config/motor calibration
calibrationConfig(device)

% Save new config and reboot
saveConfig(device)
pause(1)

% Calibrate motor
writeParameter(device, "axis0.requested_state", ODriveEnums.AxisState.AXIS_STATE_MOTOR_CALIBRATION)
pause(5)  % Wait for end of motor beep

% Save new config and reboot
saveConfig(device)

% Calibrate Encoder Offset (?)
writeParameter(device, "axis0.requested_state", ODriveEnums.AxisState.AXIS_STATE_ENCODER_OFFSET_CALIBRATION)
pause()

% Save new config and reboot
saveConfig(device)

% Can use "axis0.requested_state" == 
% ODriveEnums.AxisState.AXIS_STATE_FULL_CALIBRATION_SEQUENCE, this runs
% both motor and encoder offset calibration

