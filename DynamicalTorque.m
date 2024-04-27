% Calculate the dynamical torque of each motor

%% Preamble
close
clear
%% Add folders with functions to path
% Name of the folder to add
folderName = 'Functions';
folderName2 = 'Temporary functions';

% Get the current working directory
currentDir = pwd;

% Construct the full path to the folder
folderPath = fullfile(currentDir, folderName);
folderPath2 = fullfile(currentDir, folderName2);
% Add the folder to the MATLAB path
addpath(folderPath);
addpath(folderPath2);

%% Set up ASCII communication
baudrate = 115200;
timeout = 1;
ODriveStruct = initSerialPorts(baudrate, timeout);

%% Generate structs with ODrive modes and error codes (enum from Arduino)
% Each struct has to be passed as an argument if they need to be used in a
% function
init_ODriveEnums;
init_CDPR_Params;

% Reset encoder zeros
fieldNames = fieldnames(ODriveStruct);
for k = 1:length(fieldNames)
    fieldName = fieldNames{k}; % Current field name as a string
    currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
    setEncoderPositions(currentSerialPort);
end

for k = 1:length(fieldNames)
    fieldName = fieldNames{k}; % Current field name as a string
    currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
    setAxisState(ODriveEnums.AxisState.AXIS_STATE_CLOSED_LOOP_CONTROL, currentSerialPort)
    pause(0.1)
    setMotorTorque(1,currentSerialPort)
end
vel = zeros(4,1);
DynamicalTorques = zeros(4,1);
MotorTorques = 0.04*ones(4,1);
flag = zeros(4,1);
torqueDecrease = 0.001;
prec = .1;

while ~(all(flag))

    for k = 1:length(fieldNames)
        fieldName = fieldNames{k}; % Current field name as a string
        currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
        % Get motor velocity
        vel(k) = getEncoderVelocity(currentSerialPort)

        % If velocity is not zero and torque offset has not been logged
        if (fix(vel(k)*10^prec)/10^prec == 0) && flag(k) == 0
            flag(k) = 1;
            DynamicalTorques(k) = MotorTorques(k)
        elseif flag(k) == 0
            MotorTorques(k) = MotorTorques(k) - torqueDecrease    % Decrease torque
        end
        setMotorTorque(MotorTorques(k), currentSerialPort);
        pause(0.2)
    end
end

for k = 1:length(fieldNames)
    fieldName = fieldNames{k}; % Current field name as a string
    currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
    setAxisState(ODriveEnums.AxisState.AXIS_STATE_IDLE, currentSerialPort)
end