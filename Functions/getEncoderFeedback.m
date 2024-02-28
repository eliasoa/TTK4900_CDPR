function [pos, vel] = getEncoderFeedback(device)

% ASCII-Command for getting encoder feedback 
command = "f 0";

% Request feedback from driver
writeline(device, command)
pause(0.01)
% Read reply from driver
feedback = readline(device)        % feedback = "posistion velocity"

% Extract position and velocity from ASCII-string
pos = extractBefore(feedback," ");
vel = extractAfter(feedback, " ");

% Convert string to number
pos = str2double(pos);
vel = str2double(vel);
