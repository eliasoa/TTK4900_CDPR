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

%% Memory Allocation
N = 30;
TorqueLog   = zeros(4,N);
VelLog      = zeros(4,N);
count = 1;

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
average = zeros(4,4);


for j = 1:5

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
            pause(0.5)
        end

        % Log
        TorqueLog (:,count) = MotorTorques;
        VelLog(:,count)     = vel;
        count               = count + 1;
    end

    for k = 1:length(fieldNames)
        fieldName = fieldNames{k}; % Current field name as a string
        currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
        setAxisState(ODriveEnums.AxisState.AXIS_STATE_IDLE, currentSerialPort)
    end
    logg(:,j) = TorqueLog(:,count-1);
       % Save logs for this iteration
    save(['TorqueLog_' num2str(j) '.mat'], 'TorqueLog');
    save(['VelLog_' num2str(j) '.mat'], 'VelLog');

end
%%
average = mean(logg,2)
save('DynamicalTorqueAvg','average');

%% Plotting

save("DynamicalTorque.mat", "TorqueLog", "VelLog");



figure(1)
subplot(4,1,1)
plot(TorqueLog(1,:), VelLog(1,:))
title("ODrive0")
xlabel("Torque (Nm)", "Interpreter","latex")
ylabel("Velocity (turns/s)", "interpreter","latex")
set ( gca, 'XDir', 'reverse' )

subplot(4,1,2)
plot(TorqueLog(2,:), VelLog(2,:))
title("ODrive1")
xlabel("Torque (Nm)", "Interpreter","latex")
ylabel("Velocity (turns/s)", "interpreter","latex")
set ( gca, 'XDir', 'reverse' )

subplot(4,1,3)
plot(TorqueLog(3,:), VelLog(3,:))
title("ODrive2")
xlabel("Torque (Nm)", "Interpreter","latex")
ylabel("Velocity (turns/s)", "interpreter","latex")
set ( gca, 'XDir', 'reverse' )

subplot(4,1,4)
plot(TorqueLog(4,:), VelLog(4,:))
title("ODrive3")
xlabel("Torque (Nm)", "Interpreter","latex")
ylabel("Velocity (turns/s)", "interpreter","latex")
set ( gca, 'XDir', 'reverse' )


% Function to remove trailing zeros from each row of a matrix

