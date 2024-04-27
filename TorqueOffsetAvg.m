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
fieldNames = fieldnames(ODriveStruct);

%% Generate structs with ODrive modes and error codes (enum from Arduino)
% Each struct has to be passed as an argument if they need to be used in a
% function
init_ODriveEnums;
init_CDPR_Params;

%% Params
N = 50;
torqueIncrement = 0.01;
prec = .1;

logg = zeros(4,5);

%% Torque Calibration
% Reset encoder zeros
fieldNames = fieldnames(ODriveStruct);
for k = 1:length(fieldNames)
    fieldName = fieldNames{k}; % Current field name as a string
    currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
    setEncoderPositions(currentSerialPort);
end

for j = 1:5
    % Turn motors on
    for k = 1:length(fieldNames)
        fieldName = fieldNames{k}; % Current field name as a string
        currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
        setAxisState(ODriveEnums.AxisState.AXIS_STATE_CLOSED_LOOP_CONTROL, currentSerialPort)

    end

    % Memory Allocation
    TorqueLog   = zeros(4,N);
    VelLog      = zeros(4,N);
    MotorTorques = zeros(4,1);
    TorqueOffset = zeros(4,1);
    flag = zeros(4,1);
    vel = zeros(4,1);

    maxiter = 50;
    i = 1;
    while ~(all(flag) == 1) && i < maxiter
        % end
        % for i= 1:N

        for k = 1:length(fieldNames)
            fieldName = fieldNames{k}; % Current field name as a string
            currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
            % Get motor velocity
            vel(k) = getEncoderVelocity(currentSerialPort);

            % If velocity is not zero and torque offset has not been logged
            if (fix(vel(k)*10^prec)/10^prec ~= 0) && flag(k) == 0
                TorqueOffset(k) = MotorTorques(k);
                flag(k) = 1;
                k
                disp('Done')
                fixedvel = fix(vel(k)*10^prec)/10^prec
                normalvel = vel(k)
                disp('~~~~~~~~')
            elseif flag(k) == 0
                % k
                % i
                MotorTorques(k) = MotorTorques(k) + torqueIncrement;    % Increment torque
            end

            % if fix(vel(k)*10^prec)/10^prec == 0
            %     MotorTorques(k) = MotorTorques(k) + torqueIncrement;    % Increment torque
            % else
            %     TorqueOffset(k) = MotorTorques(k);
            % end

            % Set motor Torques
            setMotorTorque(MotorTorques(k), currentSerialPort);

            % Log Values
            TorqueLog(k,i)  = MotorTorques(k);
            VelLog(k,i)     = fix(vel(k)*10^prec)/10^prec; %vel(k);
            pause(0.01)
        end
        pause(0.3)

        i = i+1;
    end


    % Set motors to idle
    for k = 1:length(fieldNames)
        fieldName = fieldNames{k}; % Current field name as a string
        currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
        setMotorTorque(0, currentSerialPort);
    end
    logg(:,j) = TorqueLog(:,i-1);
    flag = zeros(4,1);
    pause(1)
end
% Set motors to idle
for k = 1:length(fieldNames)
    fieldName = fieldNames{k}; % Current field name as a string
    currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
    setAxisState(ODriveEnums.AxisState.AXIS_STATE_IDLE, currentSerialPort)
end



offset = mean(logg,2);
save('offset',"offset")