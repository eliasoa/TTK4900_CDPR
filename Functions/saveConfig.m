function saveConfig(device)
% saveConfig: Function for saving the currently changed configuration on a 
% ODrive S1 driver. Made by Magnus Grøterud
%
% device        : Serialport object for a driver connected to a COM port
%


% Erasing Current config
command = "ss";      % Save Config Command
writeline(device, command)

% Reboot driver
command = "sr";      % Reboot Command
writeline(device, command)

end