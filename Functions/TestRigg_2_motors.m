function TestRigg_2_motors(ODriveStruct, ODriveEnums, CDPR_Params)

% Time Params
h = 0.05;
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
while escapePressed == false
    key = waitforbuttonpress;

    
    % Check if the key press is valid
    if key == 1
        charPressed = get(gcf, 'CurrentCharacter');

        % Check which arrow key is pressed
        switch charPressed
            case 27 % Escape key
                escapePressed = true;
        end


    end
        
    % Sine torques
    T1 = 0.5*sin(2*pi*t);
    T2 = T1 - 0.5;
    T = [T1;T2];

    % Write torques
    for k = 1:length(fieldNames)
        fieldName = fieldNames{k}; % Current field name as a string
        currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
        setMotorTorque(T(k), currentSerialPort);
        disp("Motor " + string(k) + " Idle")
    end

    % Update time
    t = t + h;

end
% Terminate arrowkeyDemo
close all
for k = 1:length(fieldNames)
    fieldName = fieldNames{k}; % Current field name as a string
    currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
    setAxisState(ODriveEnums.AxisState.AXIS_STATE_IDLE, currentSerialPort)
    disp("Motor " + string(k) + " Idle")
end