function setEncoderAbsPos(absPos, device)

motor = 0;
% Command for 
command = "esl " + string(motor) + " " + string(absPos);

% Write Command to Serial Port
writeline(device, command)

end


