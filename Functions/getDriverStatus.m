function [activeErrors, errorsFound, disarmReasonsFound] = getDriverStatus(ODriveStruct, ODriveError)
% getDriverStatus: Function that checks the state of all connected ODrives
% and returns a struct with the active error codes and their respective
% values, and disarm reasons for each ODrive.
%
% Author: Elias Olsen Almenningen and ChatGPT
% Date: 28.02.2024
%
% Versions: 2 11.03.2024 Added more error flags to simplify error checking
% during program execution
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
%   errorsFound - A boolean flag indicating if any errors were found.
%   disarmReasonsFound - A boolean flag indicating if any disarm reasons were found.

%% Initialize flags and structs
errorsFound = false;
disarmReasonsFound = false;
activeErrors = struct();

%% Get all field names in the structure
fieldNames = fieldnames(ODriveStruct);
errorCommand = 'r axis0.active_errors';
disarmCommand = 'r axis0.disarm_reason';

%% Extract the field names and corresponding error values
errorNames = fieldnames(ODriveError);
errorValues = struct2array(ODriveError);

%% Iterate over each ODrive
for k = 1:length(fieldNames)
    fieldName = fieldNames{k};
    currentSerialPort = ODriveStruct.(fieldName);

    %% Fetch and store active errors
    writeline(currentSerialPort, errorCommand);
    errorCodes = str2double(readline(currentSerialPort));
    errors = struct();
    for i = 1:length(errorValues)
        if bitand(errorCodes, errorValues(i))
            errorName = errorNames{i};
            errors.(errorName) = errorValues(i);
            errorsFound = true;
        end
    end

    %% Fetch and store disarm reasons
    writeline(currentSerialPort, disarmCommand);
    disarmCodes = str2double(readline(currentSerialPort));
    disarmReasons = struct();
    for i = 1:length(errorValues)
        if bitand(disarmCodes, errorValues(i))
            disarmReasonName = errorNames{i};
            disarmReasons.(disarmReasonName) = errorValues(i);
            disarmReasonsFound = true;
        end
    end

    %% Assign to activeErrors struct
    activeErrors.(fieldName) = struct('Errors', errors, 'DisarmReasons', disarmReasons);
end
end