function calibrateMotor(device, ODriveEnums)

% Erase existing config and reboot
eraseConfig(device)

% Add new config/motor calibration
calibrationConfig(device)

% Save new config and reboot
saveConfig(device)

% Calibrate motor
writeParameter(device, "axis0.requested_state", ODriveEnums.AxisState.AXIS_STATE_MOTOR_CALIBRATION)
pause(???)  % Wait for end of motor beep

% Save new config and reboot
saveConfig(device)

% Calibrate Encoder Offset

