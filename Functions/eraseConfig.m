function eraseConfig(device)
% eraseConfig: Function for erasing current configuration on a ODrive S1
% driver. Made by Magnus Grøterud
%
% device        : Serialport object for a driver connected to a COM port
% ODriveStruct  :  


% Erasing Current config
command = "se";      % Erase Config Command
writeline(device, command)

% Reboot driver
command = "sr";      % Reboot Command
writeline(device, command)

end