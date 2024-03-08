function torqueEstimate = getTorqueEstimate(device)
% torqueEstimate = getTorqueEstimate(device)

command = "r axis0.motor.torque_estimate";
writeline(device, command)

T = readline(device);
torqueEstimate = str2double(T);

end