function mpGoesSpin(forces,ODriveEnums,ODriveStruct)
% Load parameters from CDPR_Params
R               = 0.02; % Radius of spool
% Motorsign (CHANGE IF NEEDED)
motorsign0      = -1;
motorsign1      = 1;
motorsign2      = -1;
motorsign3      = 1;
motorsigns      = [motorsign0;motorsign1;motorsign2;motorsign3];

% Load fieldNames
fieldNames      = fieldnames(ODriveStruct);
for k = 1:length(fieldNames)
    fieldName = fieldNames{k}; % Current field name as a string
    currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names

    setAxisState(ODriveEnums.AxisState.AXIS_STATE_CLOSED_LOOP_CONTROL, currentSerialPort)
    disp("Motor " + string(k) + " Active")
end

errorEncountered = false;
offsett = [0.11; -0.1; 0.175; -0.225];
offset2 = [0.13; 0.11; 0.1; 0.2];

while errorEncountered == false
    % %% Check if error
    % [~, errorsFound, disarmReasonsFound] = getDriverStatus(ODriveStruct, ODriveStruct.Error);
    % if errorsFound || disarmReasonsFound
    %     disp("Error occured, stopping motors")
    %     for k = 1:length(fieldNames)
    %         fieldName = fieldNames{k}; % Current field name as a string
    %         currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
    %         setAxisState(ODriveEnums.AxisState.AXIS_STATE_IDLE, currentSerialPort)
    %         disp("Motor " + string(k) + " Idle")
    %     end
    %     errorEncountered = true;
    %     break
    % end


    for i=1:length(forces)
        f = forces(:,i)
        T = f*R.*motorsigns*(-1);
        for k = 1:length(fieldNames)
            fieldName = fieldNames{k};
            currentSerialPort = ODriveStruct.(fieldName);
            i
            % T = T + sign(T).*offset2
            setMotorTorque(T(k), currentSerialPort)
     
        end

    end

    
    T = [0.39; -0.39; 0.39; -0.39];
    for k = 1:length(fieldNames)
        fieldName = fieldNames{k};
        currentSerialPort = ODriveStruct.(fieldName);
        setMotorTorque(T(k), currentSerialPort)
    end
    errorEncountered = true;


end





end