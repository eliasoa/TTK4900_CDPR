function motorPosTest(ODriveStruct,ODriveEnums)

userInput = input('Enter position setpoint  [-1, 1]: ', 's'); % 's' for string input
x_target = str2double(userInput); % Convert to number
disp(['You entered the setpoint: ', num2str(x_target)]);

errorEncountered = false;
fieldNames = fieldnames(ODriveStruct);
motorPos = [0;0];


for k = 1:length(fieldNames)
    fieldName = fieldNames{k}; % Current field name as a string
    currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
    setAxisState(ODriveEnums.AxisState.AXIS_STATE_CLOSED_LOOP_CONTROL, currentSerialPort)
    disp("Motor " + string(k) + " Active")
end

while true
    %% If an error occurs, stop motors
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
    %% Get motor position from encoders
    for k = 1:length(fieldNames)
        ODrive = ODriveStruct.(fieldNames{k});
        motorPos(k) = getMotorPosition(ODrive);
    end
    x_r = motorPos(1);
    x_l = motorPos(2);
    x_current = (x_r - x_l) / 2;
    e = x_target - x_current % Positive error, go right


    %% Tension logic
    % Given values
    t_ref = -0.2;
    t_min = -0.1;  % Less tension
    t_max = -0.3;  % More tension

    Kp = .5; % Tunable gain

    % Calculate tension adjustment based on error
    t = e * Kp;

    % Calculate tension for each motor, ensuring it stays within limits
    T_left = max(min(t_ref - t, t_min), t_max);
    T_right = max(min(t_ref + t, t_min), t_max);

    % Tension vector to be sent to motors
    T = [T_right; T_left]

    for k = 1:length(fieldNames)
        ODrive = ODriveStruct.(fieldNames{k});
        setMotorTorque(T(k), ODrive);
    end
    
    if abs(e) < 0.05
        pause(3);
        break;
    end


end

if~errorEncountered
    for k = 1:length(fieldNames)
        fieldName = fieldNames{k}; % Current field name as a string
        currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
        setAxisState(ODriveEnums.AxisState.AXIS_STATE_IDLE, currentSerialPort)
        disp("Motor " + string(k) + " Idle")
    end
end


end