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

%% Set up ASCII communication
baudrate = 115200;
timeout = 1;
ODriveStruct = initSerialPorts(baudrate, timeout);

%% Generate structs with ODrive modes and error codes (enum from Arduino)
% Each struct has to be passed as an argument if they need to be used in a
% function

init_ODriveEnums;
init_CDPR_Params;

%% How to access each driver in the driver struct
% Assuming you have the structure serialPortsStruct with dynamic field names

% % Get all field names in the structure
% fieldNames = fieldnames(ODriveStruct);
%
% % Iterate over each field in the structure
% for k = 1:length(fieldNames)
%     fieldName = fieldNames{k}; % Current field name as a string
%     currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
%
%     % Now you can use currentSerialPort as needed
%     setAxisState(1,currentSerialPort);
% end
%% Main program
while true
    userInput = input('Enter a number or type "exit" to stop: ', 's'); % 's' for string input
    % userInput = '2';
    if strcmp(userInput, 'exit')
        clc
        disp('Exiting...');
        break; % Exit the loop
    elseif all(isstrprop(userInput, 'digit')) % Check if all characters are digits
        number = str2double(userInput); % Convert to number
        disp(['You entered the number: ', num2str(number)]);

        % Switch-case structure to handle different numeric inputs
        switch number
            case 1
                clc
                disp("Move with cursor mode")
                disp("Press Esc to exit")
                arrowKey_manual_control(ODriveStruct, ODriveEnums, CDPR_Params)
                clc
            case 2
                clc
                % disp("Path following mode")
                % pathFollowingMode(ODriveStruct, ODriveEnums);
                fieldNames = fieldnames(ODriveStruct);

                % Iterate over each field in the structure
                for k = 1:length(fieldNames)
                    fieldName = fieldNames{k}; % Current field name as a string
                    currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names

                    % Now you can use currentSerialPort as needed
                    setEncoderPositions(currentSerialPort);
                end
                
            case 3
                % disp("Bounce my ballz mode")
                motorPosTest(ODriveStruct, ODriveEnums);
                
            case 4
                clc
                disp("Testing mode")
                errorCheckAndHandling(ODriveStruct, ODriveError);
            case 5
                clc
                disp("Testing: 4 motors")
                TestRigg_4_motors_VelControl(ODriveStruct, ODriveEnums, CDPR_Params)
            case 6
                clc
                % calibrateMotor(ODriveStruct.ODrive0,ODriveEnums)
                
            otherwise
                disp('Input number does not match any function.');
                clc
        end
    else
        disp('Invalid input. Please enter a number or "exit".');
    end
end
