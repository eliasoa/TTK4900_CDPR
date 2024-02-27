function serialPortsStruct = initSerialPorts(baudrate, timeout)
% Get the number of motor controllers connected and their COM ports
motorDrivers = getNumberOfAxisAndCOMPortNumber();
numberOfDrivers = size(motorDrivers, 1);

% Initialize a structure and a cell array for storing serial port objects
serialPortsStruct = struct();
serialPortsArray = cell(numberOfDrivers, 1);

for k = 1:numberOfDrivers
    driverName = strcat('ODrive', num2str(motorDrivers(k, 1)));
    driverPort = serialport(strcat('COM', num2str(motorDrivers(k, 2))), baudrate, "Timeout", timeout);
    
    % Store the serialport object in both the structure and the array
    serialPortsStruct.(driverName) = driverPort;
    serialPortsArray{k} = driverPort;
end
end