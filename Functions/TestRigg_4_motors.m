function TestRigg_4_motors(ODriveStruct, ODriveEnums, CDPR_Params)


%% Initialization 


% Get all field names in the ODrive struct
fieldNames = fieldnames(ODriveStruct);

for k = 1:length(fieldNames)
    fieldName = fieldNames{k}; % Current field name as a string
    currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names

    setAxisState(ODriveEnums.AxisState.AXIS_STATE_CLOSED_LOOP_CONTROL, currentSerialPort)
    disp("Motor " + string(k) + " Active")
end

%% Initialize variables
x   = 0;                            % Desired x-position
y   = 0;                            % Desired y-position
phi = 0;                            % Desired phi-angle [radians]
escapePressed = false;              % Initialize termination button (Press Esc to )
errorEncountered = false;           % Initialize error counter bit
        
posIncrement    = 0.5;              % Position Increment each arrow click
angleIncrement  = 2;                % Angle increment each arrow click

% Initialize full states
s           = [q0;dq0;zeros(6,1)];  % States        ( MÅ HENTE INITIAL CONDITIONS FRA ET STED, CDPR_PARAMS???  
e           = zeros(6,1);
e_int       = zeros(6,1);



%% Control Loop
while escapePressed == false && errorEncountered == false
    %% Check if error
    [~, errorsFound, disarmReasonsFound] = getDriverStatus(ODriveStruct, ODriveEnums.Error);
    if errorsFound || disarmReasonsFound  
        disp("Error occured, stopping motors")
        for k = 1:length(fieldNames)
            fieldName = fieldNames{k}; % Current field name as a string
            currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
            setAxisState(ODriveEnums.AxisState.AXIS_STATE_IDLE, currentSerialPort)
            disp("Motor " + string(k) + " Idle")
        end
        errorEncountered = true;
        break
    end

    %% Check Keys and update reference pose
    key = waitforbuttonpress; % Muligens fjerne denne?? Må oppdatere selv om det ikke kommer knappetrykk... 
    if key == 1
        charPressed = get(gcf, 'CurrentCharacter');

        % Check which arrow key is pressed
        switch charPressed
            case 28 % Left arrow
                x = x - posIncrement;
            case 29 % Right arrow
                x = x + posIncrement;
            case 30 % Up arrow
                y = y + posIncrement;
            case 31 % Down arrow
                y = y - posIncrement;
            case 27 % Escape key
                escapePressed = true;
            case 'a' % 'a' key
                phi = phi + deg2rad(angleIncrement);
            case 'd' % 'd' key
                phi = phi - deg2rad(angleIncrement);
        end
    end

    %% Estimate Cable Lengths
    for k = 1:length(fieldNames)
        fieldName = fieldNames{k}; % Current field name as a string
        currentSerialPort = ODriveStruct.(fieldName);                       % Access the current serial port using dynamic field names

        % flush(currentSerialPort)
        [pos, vel] = getEncoderFeedback(currentSerialPort);                 % Get angular position and velocity from encoder

        if (-1)^(k) == -1                                                   % Determine motorsign (Even or odd, can change this)!!!!!!!!!!!!!!!!!!!
            motorsign = -1;
        else
            motorsign = 1;
        end

        l(k)        = encoder2cableLen(pos,l0(k),R, motorsign);             % Estimate cable length
        l_dot(k)    = encoder2cableVel(vel, CDPR_Params, motorsign);        % Estimated Rate of change of cable
    end
    %% Controller


       

    


    


end







end
