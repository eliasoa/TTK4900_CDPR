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
    [activeErrors, errorsFound, disarmReasonsFound] = getDriverStatus(ODriveStruct, ODriveError);
    if ~errorsFound && ~disarmReasonsFound
        disp("No errors :)");
    end
    displayErrors(activeErrors);
    stopFlag = handleErrors(ODriveStruct, activeErrors);
    if stopFlag
        break;
    end


    % Check if errors are cleared
    [~, errorsFound, disarmReasonsFound] = getDriverStatus(ODriveStruct, ODriveError);
    if errorsFound || disarmReasonsFound
        disp("Errors or disarmreasons found, fix them and try again");
    else
        disp("Errors cleares sucsessfully :)");
        break;
    end
end
end