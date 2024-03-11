function TestRigg_2_motors(ODriveStruct, ODriveEnums, CDPR_Params)

% Time Params
h = 0.005;
t = 0;
escapePressed = false;  % Initialize termination button (Press Esc to )

% Get all field names in the ODrive struct
fieldNames = fieldnames(ODriveStruct);


for k = 1:length(fieldNames)
    fieldName = fieldNames{k}; % Current field name as a string
    currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names

    setAxisState(ODriveEnums.AxisState.AXIS_STATE_CLOSED_LOOP_CONTROL, currentSerialPort)
    disp("Motor " + string(k) + " Active")
end

errorEncountered = false;
while escapePressed == false
    key = waitforbuttonpress;

    % Check if error
    [~, errorsFound, disarmReasonsFound] = getDriverStatus(ODriveStruct, ODriveEnums.Error);
    if errorsFound || disarmReasonsFound
        errorEncountered = true;
        disp("Error occured, stopping motors")
        for k = 1:length(fieldNames)
            fieldName = fieldNames{k}; % Current field name as a string
            currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
            setAxisState(ODriveEnums.AxisState.AXIS_STATE_IDLE, currentSerialPort)
            disp("Motor " + string(k) + " Idle")
        end
        break
    end

    % Check if the key press is valid
    if key == 1
        charPressed = get(gcf, 'CurrentCharacter');

        % Check which arrow key is pressed
        switch charPressed
            case 27 % Escape key
                escapePressed = true;
            case 30 % Up arrow
                % Sine torques
                torque = 0.5;
                freq = 100;
                T1 =  torque*sin(freq*t) - torque;
                T2 = -torque*sin(freq*t) - torque;
                T = [T1;T2];

                % Write torques
                for k = 1:length(fieldNames)
                    fieldName = fieldNames{k}; % Current field name as a string
                    currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
                    setMotorTorque(T(k), currentSerialPort);
                    disp("ODrive" + string(k-1) + " Torque set point: " + T(k))
                end
                % Update time
                t = t + h;
            case 31 % Down arrow
                T = [-1;-1]*0.5;
                for k = 1:length(fieldNames)
                    fieldName = fieldNames{k}; % Current field name as a string
                    currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
                    setMotorTorque(T(k), currentSerialPort);
                    disp("ODrive" + string(k-1) + " Torque set point: " + T(k))
                end


        end
    end
end

close all;
if~errorEncountered
    for k = 1:length(fieldNames)
        fieldName = fieldNames{k}; % Current field name as a string
        currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
        setAxisState(ODriveEnums.AxisState.AXIS_STATE_IDLE, currentSerialPort)
        disp("Motor " + string(k) + " Idle")
    end
end
end