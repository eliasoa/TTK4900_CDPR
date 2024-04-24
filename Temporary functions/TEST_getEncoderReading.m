function [pos, vel] = TEST_getEncoderReading(ODriveStruct)
% Preallocate vector
pos = zeros(4,1);
vel = zeros(4,1);

odrv0 = ODriveStruct.ODrive0;
odrv1 = ODriveStruct.ODrive1;
odrv2 = ODriveStruct.ODrive2;
odrv3 = ODriveStruct.ODrive3;

% ASCII-Command for getting encoder feedback on the form [pos vel]
command = "f 0";

% This only works when 4 ODrives are connected

%% ODrive0
% Request feedback from driver
writeline(odrv0, command);
% Read reply from driver
feedback = readline(odrv0);
% Extract position and velocity from ASCII-string
pos(1) = extractBefore(feedback, " ");
vel(1) = extractAfter(feedback, " ");

%% ODrive1
% Request feedback from driver
writeline(odrv1, command);
% Read reply from driver
feedback = readline(odrv1);
% Extract position and velocity from ASCII-string
pos(2) = extractBefore(feedback, " ");
vel(2) = extractAfter(feedback, " ");

%% ODrive2
% Request feedback from driver
writeline(odrv2, command);
% Read reply from driver
feedback = readline(odrv2);
% Extract position and velocity from ASCII-string
pos(3) = extractBefore(feedback, " ");
vel(3) = extractAfter(feedback, " ");
%% ODrive3
% Request feedback from driver
writeline(odrv3, command);
% Read reply from driver
feedback = readline(odrv3);
% Extract position and velocity from ASCII-string
pos(4) = extractBefore(feedback, " ");
vel(4) = extractAfter(feedback, " ");
end