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
while escapePressed == false
    key = waitforbuttonpress;

    writeline(currentSerialPort,"axis0.torque")
    % Check if the key press is valid
    if key == 1
        charPressed = get(gcf, 'CurrentCharacter');

        % Check which arrow key is pressed
        switch charPressed
            case 27 % Escape key
                escapePressed = true;
            case 30 % Up arrow
                T = [-1;-1]*0.8;
                for k = 1:length(fieldNames)
                    fieldName = fieldNames{k}; % Current field name as a string
                    currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
                    setMotorTorque(T(k), currentSerialPort);
                    disp("Motor " + string(k) + " Idle")
                end
            case 31 % Down arrow
                T = [-1;-1]*0.0;
                for k = 1:length(fieldNames)
                    fieldName = fieldNames{k}; % Current field name as a string
                    currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
                    setMotorTorque(T(k), currentSerialPort);
                    disp("Motor " + string(k) + " Idle")
                end
                
                % % Sine torques
                % T1 = 0.2*sin(2*pi*t) - 0.2;
                % T2 = 0.22*cos(2*pi*t) + 0.04;
                % T = [0;T2];
                % 
                % % Write torques
                % for k = 1:length(fieldNames)
                %     fieldName = fieldNames{k}; % Current field name as a string
                %     currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
                %     setMotorTorque(T(k), currentSerialPort);
                %     disp("Motor " + string(k) + " Idle")
                % end
                % % Update time
                % t = t + h;
        end
    end



end
close all;
for k = 1:length(fieldNames)
    fieldName = fieldNames{k}; % Current field name as a string
    currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
    setAxisState(ODriveEnums.AxisState.AXIS_STATE_IDLE, currentSerialPort)
    disp("Motor " + string(k) + " Idle")
end