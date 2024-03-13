function pos = getMotorPosition(device)
command = "r axis0.pos_estimate";
writeline(device, command);
pos = readline(device);
end







