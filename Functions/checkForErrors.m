function isError = checkForErrors(ODriveStruct, ODriveEnums)
fieldNames = fieldnames(ODriveStruct);
isError = false;

[~, errorsFound, disarmReasonsFound] = getDriverStatus(ODriveStruct, ODriveEnums.Error);
if errorsFound || disarmReasonsFound
    disp("Error occured, stopping motors")
    for k = 1:length(fieldNames)
        fieldName = fieldNames{k}; % Current field name as a string
        currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
        setAxisState(ODriveEnums.AxisState.AXIS_STATE_IDLE, currentSerialPort)
        disp("Motor " + string(k) + " Idle")
    end
    isError = true;
end