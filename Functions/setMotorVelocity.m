function setMotorVelocity(velocity, torque_ff, COM_Port,baudrate)

device = serialport(COM_Port, baudrate);

motor = 0;
command = "v " + string(motor) + " " + string(velocity) + " " + string(torque_ff);

% Write Command to Serial Port
writeline(device, command)

end


