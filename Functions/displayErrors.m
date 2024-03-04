function displayErrors(activeErrors)
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