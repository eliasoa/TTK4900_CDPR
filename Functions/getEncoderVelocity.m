function vel = getEncoderVelocity(device)
% ASCII-Command for getting encoder feedback 
command = "f 0";

% Request feedback from driver
% flush(device)
writeline(device, command)

% Read reply from driver
feedback = readline(device);        % feedback = "posistion velocity"

% Extract position and velocity from ASCII-string
vel = extractAfter(feedback, " ");

% Convert string to number
vel = str2double(vel);

end


