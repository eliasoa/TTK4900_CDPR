function setMotorPosition(pos, v_lim, tlim, device)
% pos: Commanded Motor Position [Turns]

motor = 0;
command = "q " + string(motor) + " " + string(pos) + " " + string(v_lim) + " " + string(tlim);

% Write Command to motor driver
writeline(device, command)