function pos = getEncoderPosition(device)

% ASCII-Command for getting encoder feedback 
command = "f 0";

% Request feedback from driver
flush(device)
writeline(device, command)

% Read reply from driver
feedback = readline(device);        % feedback = "posistion velocity"

% Extract position and velocity from ASCII-string
pos = extractBefore(feedback," ");

% Convert string to number
pos = str2double(pos);

