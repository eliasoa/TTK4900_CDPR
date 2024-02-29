function stopFlag = handleErrors(ODriveStruct, activeErrors)
    % Function to handle errors for each ODrive
    % Parameters:
    %   ODriveStruct - A struct where each field corresponds to a connected
    %                  ODrive, represented by a serial port object.
    %   activeErrors - A struct containing active errors and disarm reasons for
    %                  each ODrive.
    %
    % Returns:
    %   stopFlag - Boolean flag indicating whether to stop the program due to unhandled errors.

    ODrives = fieldnames(ODriveStruct);
    stopFlag = true; % Default to true as a safety measure

    % Check if there are any errors
    if isempty(fieldnames(activeErrors))
        disp("No errors to handle :)");
        stopFlag = false;
        return;
    end

    % Iterate through each ODrive and check for errors
    for k = 1:length(ODrives)
        fieldName = ODrives{k};
        if isfield(activeErrors, fieldName)
            errorsStruct = activeErrors.(fieldName).Errors;
            disarmStruct = activeErrors.(fieldName).DisarmReasons;

            if ~isempty(fieldnames(errorsStruct)) || ~isempty(fieldnames(disarmStruct))
                disp(['Errors or disarm reasons detected in ODrive: ', fieldName]);
                userInput = input('Clear all errors? y/n: ', 's');
                switch userInput
                    case 'y'
                        command = 'sc';
                        currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port
                        writeline(currentSerialPort, command); % Send clear command
                        stopFlag = false; % Errors cleared, no need to stop

                    case 'n'
                        disp("Errors not cleared, stopping program");
                        stopFlag = true;
                        return; % Early return as user chose not to clear errors

                    otherwise
                        disp("Invalid input");
                        return; % Early return due to invalid input
                end
            end
        end
    end

    % If no errors were detected in any ODrive
    if stopFlag
        disp("No errors to handle in any ODrive :)");
        stopFlag = false;
    end
end
