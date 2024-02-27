%% Preamble
close 
clear
%% Add folders with functions to path
% Name of the folder to add
folderName = 'Functions';

% Get the current working directory
currentDir = pwd;

% Construct the full path to the folder
folderPath = fullfile(currentDir, folderName);

% Add the folder to the MATLAB path
addpath(folderPath);

%% Set up ASCII communications
baudrate = 115200;
timeout = 1;
ODriveStruct = initSerialPorts(baudrate, timeout);

%% Axis state
% Assuming you have the structure serialPortsStruct with dynamic field names

% % Get all field names in the structure
% fieldNames = fieldnames(serialPortsStruct);
% 
% % Iterate over each field in the structure
% for k = 1:length(fieldNames)
%     fieldName = fieldNames{k}; % Current field name as a string
%     currentSerialPort = serialPortsStruct.(fieldName); % Access the current serial port using dynamic field names
% 
%     % Now you can use currentSerialPort as needed
%     setAxisState(1,currentSerialPort);
% end