function [activeErrors] = getDriverStatus(ODriveStruct, ODriveError)
    % getDriverStatus: Function that checks the state of all connected ODrives
    % and returns a struct with the active error codes and their respective
    % values, and disarm reasons for each ODrive. It also prints the errors.
    %
    % Author: Elias Olsen Almenningen and ChatGPT
    % Date: 28.02.2024
    %
    % Parameters:
    %   ODriveStruct - A struct where each field corresponds to a connected
    %                  ODrive, represented by a serial port object.
    %   ODriveError  - A struct mapping error names and disarm reasons to their 
    %                  respective error code values (as integers).
    %
    % Returns:
    %   activeErrors - A struct where each field corresponds to an ODrive
    %                  device. Each field contains nested structs with the
    %                  names of active errors and disarm reasons along with 
    %                  their respective error code values.
    %
    % Usage:
    %   [errors] = getDriverStatus(myODrives, myODriveErrors);
    %   Here, 'myODrives' is an example of ODriveStruct, and 'myODriveErrors'
    %   is an example of ODriveError struct.

    %% Get all field names in the structure
    fieldNames = fieldnames(ODriveStruct);

    errorCommand = 'r axis0.active_errors';
    disarmCommand = 'r axis0.disarm_reason';
    % Extract the field names and corresponding error values
    errorNames = fieldnames(ODriveError);
    errorValues = struct2array(ODriveError);

    % Initialize the activeErrors struct
    activeErrors = struct();

    % Iterate over each field in the structure
    for k = 1:length(fieldNames)
        fieldName = fieldNames{k}; % Current field name as a string
        currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
        
        % Fetch and store active errors
        writeline(currentSerialPort, errorCommand);
        errorCodes = str2double(readline(currentSerialPort)); % Read error states
        errors = struct(); % Initialize errors struct for current ODrive
        for i = 1:length(errorValues)
            if bitand(errorCodes, errorValues(i))
                errorName = errorNames{i};
                errors.(errorName) = errorValues(i);
            end
        end

        % Fetch and store disarm reason
        writeline(currentSerialPort, disarmCommand);
        disarmCodes = str2double(readline(currentSerialPort)); % Read disarm reason
        disarmReasons = struct(); % Initialize disarmReasons struct for current ODrive
        for i = 1:length(errorValues)
            if bitand(disarmCodes, errorValues(i))
                disarmReasonName = errorNames{i};
                disarmReasons.(disarmReasonName) = errorValues(i);
            end
        end

        % Assign errors and disarm reasons to the current field in activeErrors
        activeErrors.(fieldName) = struct('Errors', errors, 'DisarmReasons', disarmReasons);
    end

    %% Display the active errors for each ODrive
    disp('Active Errors and Disarm Reasons for each ODrive:');

    % Iterate over each field in the activeErrors struct
    fieldNames = fieldnames(activeErrors);
    for i = 1:length(fieldNames)
        fieldName = fieldNames{i};
        disp(['ODrive: ', fieldName]);

        % Display Errors
        errorsStruct = activeErrors.(fieldName).Errors;
        if isempty(fieldnames(errorsStruct))
            disp('  No active errors.');
        else
            disp('  Errors:');
            nestedFieldNames = fieldnames(errorsStruct);
            for j = 1:length(nestedFieldNames)
                nestedFieldName = nestedFieldNames{j};
                errorValue = errorsStruct.(nestedFieldName);
                disp(['    ', nestedFieldName, ' - Value: ', num2str(errorValue)]);
            end
        end

        % Display Disarm Reasons
        disarmStruct = activeErrors.(fieldName).DisarmReasons;
        if isempty(fieldnames(disarmStruct))
            disp('  No disarm reasons.');
        else
            disp('  Disarm Reasons:');
            nestedFieldNames = fieldnames(disarmStruct);
            for j = 1:length(nestedFieldNames)
                nestedFieldName = nestedFieldNames{j};
                disarmReasonValue = disarmStruct.(nestedFieldName);
                disp(['    ', nestedFieldName, ' - Value: ', num2str(disarmReasonValue)]);
            end
        end
    end
end
