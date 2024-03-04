function errorCheckAndHandling(ODriveStruct, ODriveError)



ODrives = fieldnames(ODriveStruct);

while true
    activeErrors = getDriverStatus(ODriveStruct, ODriveError);
    displayErrors(activeErrors);
    handleErrors(ODriveStruct, activeErrors);


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