function [activeErrors] = getDriverStatus(ODriveStruct, ODriveError)
% getDriverStatus: Function that checks the state of all connected ODrives
% and returns a struct with the active error codes and their respective
% values for each ODrive. It prints the errors aswell
%
% Author: Elias Olsen Almenningen and ChatGPT
% Date: 28.02.2024
%
%
% Parameters:
%   ODriveStruct - A struct where each field corresponds to a connected
%                  ODrive, represented by a serial port object.
%   ODriveError  - A struct mapping error names to their respective error
%                  code values (as integers).
%
% Returns:
%   activeErrors - A struct where each field corresponds to an ODrive
%                  device. Each field contains a nested struct with the
%                  names of active errors and their respective error code
%                  values.
%
% Usage:
%   [errors] = getDriverStatus(myODrives, myODriveErrors);
%   Here, 'myODrives' is an example of ODriveStruct, and 'myODriveErrors'
%   is an example of ODriveError struct.

%% Get all field names in the structure
fieldNames = fieldnames(ODriveStruct);

command = 'r axis0.active_errors';
% Extract the field names and corresponding error values
errorNames = fieldnames(ODriveError);
errorValues = struct2array(ODriveError);

% Initialize the activeErrors struct
activeErrors = struct();

% Iterate over each field in the structure
for k = 1:length(fieldNames)
    fieldName = fieldNames{k}; % Current field name as a string
    currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
    writeline(currentSerialPort, command);
    errorCodes = str2double(readline(currentSerialPort)); % Read error states

    % Initialize errors struct for current ODrive
    errors = struct();

    % Check for each error
    for i = 1:length(errorValues)
        if bitand(errorCodes, errorValues(i))
            errorName = errorNames{i};
            errors.(errorName) = errorValues(i);
        end
    end

    % Assign errors struct to the current field in activeErrors
    activeErrors.(fieldName) = errors;
end
%% Display the active errors for each ODrive
disp('Active Errors for each ODrive:');

% Iterate over each field in the activeErrors struct
fieldNames = fieldnames(activeErrors);
for i = 1:length(fieldNames)
    fieldName = fieldNames{i};
    disp(['ODrive: ', fieldName]);

    % Get the nested struct for this field
    nestedStruct = activeErrors.(fieldName);

    % Check if the nested struct is empty (no errors)
    if isempty(fieldnames(nestedStruct))
        disp('  No active errors.');
    else
        % Iterate over each field in the nested struct
        nestedFieldNames = fieldnames(nestedStruct);
        for j = 1:length(nestedFieldNames)
            nestedFieldName = nestedFieldNames{j};
            errorValue = nestedStruct.(nestedFieldName);
            disp(['  Error: ', nestedFieldName, ' - Value: ', num2str(errorValue)]);
        end
    end
end
end