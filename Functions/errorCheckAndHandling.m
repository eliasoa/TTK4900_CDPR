function errorCheckAndHandling(ODriveStruct, ODriveError)
% Function that checks for errors on the ODrive drivers and handles the
% errors
%
% Author: Elias Olsen Almenningen and ChatGPT
% Date: 04.03.2024
%
% Parameters:
%   ODriveStruct - A struct where each field corresponds to a connected
%                  ODrive, represented by a serial port object.
%   ODriveError  - A struct mapping error names and disarm reasons to their
%                  respective error code values (as integers).


ODrives = fieldnames(ODriveStruct);

while true
    activeErrors = getDriverStatus(ODriveStruct, ODriveError);
    displayErrors(activeErrors);
    stopFlag = handleErrors(ODriveStruct, activeErrors);
    if stopFlag
        break;
    end

    % Check if errors are cleared
    activeErrors = getDriverStatus(ODriveStruct, ODriveError);

    for k = 1:length(ODrives)
        fieldName = ODrives{k};
        if isfield(activeErrors, fieldName)
            errorsStruct = activeErrors.(fieldName).Errors;
            disarmStruct = activeErrors.(fieldName).DisarmReasons;

            if isempty(fieldnames(errorsStruct)) && isempty(fieldnames(disarmStruct))
                disp("Errors succsessfully cleared")
                return;
            else 
                disp("Errors not cleared, fix errors and try again")
            end
        end
    end
end
end

