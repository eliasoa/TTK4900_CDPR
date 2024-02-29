function testingMode(ODriveStruct, ODriveError)
% Function made for testing functions in development
% The following functinos were made in this function:
% - getDriverStatus
% - 

fieldNames = fieldnames(ODriveStruct);
numberOfMotors = numel(fieldNames);

for k = 1:numberOfMotors
    motorName = ['ODrive', num2str(k-1)]; % Generate the variable name
    if isfield(ODriveStruct, motorName)
        eval([motorName ' = ODriveStruct.' motorName ';']);
    end
end

%% Read errors
command = 'r axis0.active_errors';
writeline(ODrive0,command)
errorCodes = str2double(readline(ODrive0))


% Extract the field names and corresponding error values
errorNames = fieldnames(ODriveError);
errorValues = struct2array(ODriveError);

% Check for each error
for i = 1:length(errorValues)
    if bitand(errorCodes, errorValues(i))
        disp(['Error present: ', errorNames{i}]);
    end
end


end