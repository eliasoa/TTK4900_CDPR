function ODriveStruct = initSerialPorts(baudrate, timeout)
% serialPortsStruct: Function that initilizes the SerialPortsStruct
% containing the ODrive COM ports connected to the PC. It always ensures
% that the ODrive connected to COM4 is ODrive0, COM5 is ODrive1 and so on.
% It is only guaranteed to work on the development pc.
%
% Author: Elias Olsen Almenningen and ChatGPT
% Date: 28.02.2024
%
% Versions:
%
% Parameters:
%   baudrate        - The speed of communication over the serial port, 
%                     specified in bits per second (bps). 
%
%   timeout         - The time in seconds to wait for a completed read 
%                     or write operation on the serial port. If a timeout 
%                     occurs, the read or write operation terminates.
%
% Returns:
%   ODriveStruct    - A struct containing serial port objects for each 
%                     ODrive motor controller. Each field of the struct is
%                     named according to the ODrive identifier (e.g., 
%                     'ODrive0', 'ODrive1', etc.) and contains the serial 
%                     port object associated with that particular ODrive.
%

% Get the number of motor controllers connected and their COM ports
motorDrivers = getNumberOfAxisAndCOMPortNumber();
numberOfDrivers = size(motorDrivers, 1);

% Initialize a structure and a cell array for storing serial port objects
ODriveStruct = struct();
serialPortsArray = cell(numberOfDrivers, 1);

for k = 1:numberOfDrivers
    driverName = strcat('ODrive', num2str(k - 1)); % ODrive indices start at 0
    comPortNum = 3 + k; % Assign COM port starting from COM4 for ODrive0
    driverPort = serialport(strcat('COM', num2str(comPortNum)), baudrate, "Timeout", timeout);

    % Store the serialport object in both the structure and the array
    ODriveStruct.(driverName) = driverPort;
    serialPortsArray{k} = driverPort;
end
end
