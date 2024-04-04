function TestRigg_4_motors(ODriveStruct, ODriveEnums, CDPR_Params)

%% Parameters
R           = CDPR_Params.Gen_Params.SPOOL_RADIUS;        % Radius of spool
a           = CDPR_Params.SGM.FrameAP;                    % Frame Anchor Points
b           = CDPR_Params.SGM.BodyAP.RECTANGLE;           % Body Anchor Points
motorsigns  = CDPR_Params.Gen_Params.MOTOR_SIGNS;         % Signs determining positive rotational direction 
m_p         = CDPR_Params.Gen_Params.Platform_mass;       % Mass of MP
f_min = 0.3/R;
f_max = 0.5/R;
f_ref = 0.4/R;

%% Initialize variables
x   = 0;                            % Desired x-position
y   = 0;                            % Desired y-position
phi = 0;                            % Desired phi-angle [radians]
escapePressed = false;              % Initialize termination button (Press Esc to )
errorEncountered = false;           % Initialize error counter bit
        
posIncrement    = 0.5;              % Position Increment each arrow click
angleIncrement  = 2;                % Angle increment each arrow click

% Initialize full states
q0          = [0;0;0];              % Initial Pose
dq0         = [0;0;0];              % Initial Velocity
l0          = [0.3;0.3;0.3;0.3];

% s           = [q0;dq0;zeros(6,1)];  % State Vector       
% e           = zeros(6,1);           % Memory Allocation for pose error
% e_int       = zeros(6,1);           % Memory Allocation for integral of error
f_prev      = zeros(4,1);           % Memory Allocation for prev cable forces

%% Initialization 
% Get all field names in the ODrive struct
fieldNames = fieldnames(ODriveStruct);

for k = 1:length(fieldNames)
    fieldName = fieldNames{k}; % Current field name as a string
    currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names

    setAxisState(ODriveEnums.AxisState.AXIS_STATE_CLOSED_LOOP_CONTROL, currentSerialPort)
    disp("Motor " + string(k) + " Active")
end

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
    % key = waitforbuttonpress; % Muligens fjerne denne?? Må oppdatere selv om det ikke kommer knappetrykk... 
    % if key == 1
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
    % end

    %% Estimate Cable Lengths
    for k = 1:length(fieldNames)
        fieldName = fieldNames{k}; % Current field name as a string
        currentSerialPort = ODriveStruct.(fieldName);                       % Access the current serial port using dynamic field names

        % flush(currentSerialPort)
        pos = getEncoderPosition(currentSerialPort);
        vel = getEncoderVelocity(currentSerialPort);                 % Get angular position and velocity from encoder
   
        l(k)        = encoder2cableLen(pos,l0(k),R, motorsigns(k));             % Estimate cable length
        l_dot(k)    = encoder2cableVel(vel, CDPR_Params, motorsigns(k));        % Estimated Rate of change of cable
    end
    %% Controller
    % Calculate current and desired pose of the platform
    q           = DirectKinematics_V2(a,b,l);   % Measured pose
    q_d         = [x;y;phi];                    % Desired pose    

    % Calculate Structure Matrix
    A = WrenchMatrix_V2(a,b,q);
    % A_t = A';
    % A_t_pseudo = pinv(A_t);

    % Calculate current velocity of the platform 
    % q_dot       = -A_t_pseudo*l_dot;             % Estimated velocity (Not currently in use)
    
    % Calculate Errors
    e = q-q_d;
    
    Kp = eye(3);  % lol
    % K_d = 1;

    % Desired wrench (TESTE KONTROLLER)
    w_c = -Kp*e;

    [f, flag] = Optimal_ForceDistributions(A,w_c,m_p,f_min,f_max,f_ref, f_prev);

    % Assume ideal world
    
    T = f*R;
    
    % Write torque to motor drivers (YOOOO: CHECK POSITIV REGNING)
    for k = 1:length(fieldNames)
        fieldName = fieldNames{k}; % Current field name as a string
        currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
        setMotorTorque(T(k), currentSerialPort)
    end
    
    % Save previous cable forces
    f_prev = f;
    
    % Update Integral effect (later)
    
end

end
