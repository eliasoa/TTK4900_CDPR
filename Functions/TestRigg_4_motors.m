function TestRigg_4_motors(ODriveStruct, ODriveEnums, CDPR_Params)

%% Parameters
R           = CDPR_Params.Gen_Params.SPOOL_RADIUS;        % Radius of spool
a           = CDPR_Params.SGM.FrameAP;                    % Frame Anchor Points
b           = CDPR_Params.SGM.BodyAP.TRAPEZOID;           % Body Anchor Points
motorsigns  = CDPR_Params.Gen_Params.MOTOR_SIGNS;         % Signs determining positive rotational direction
% m_p         = CDPR_Params.Gen_Params.Platform_mass;       % Mass of MP
f_min = 5;
f_max = 60;
f_ref = 25;

%% Initialize variables
x   = 0.1;                        % Desired x-position
y   = 0;                            % Desired y-position
phi = 0;                            % Desired phi-angle [radians]

q_d         = [x;y;phi];            % Desired pose

errorEncountered = false;           % Initialize error counter bit

[~,l0,~] = CDPR_InverseKinematics_V2([0;0;0], a, b)

f_prev      = [0.39; -0.39; 0.39; -0.39];   % Memory Allocation for prev cable forces

l           = zeros(4,1);                   % Memory Allocation for cable lengths
l_dot       = zeros(4,1);                   % Memory Allocation for cable velocities

%% Initialization
% Get all field names in the ODrive struct
fieldNames = fieldnames(ODriveStruct);

% for k = 1:length(fieldNames)
%     fieldName = fieldNames{k}; % Current field name as a string
%     currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
%
%     setAxisState(ODriveEnums.AxisState.AXIS_STATE_CLOSED_LOOP_CONTROL, currentSerialPort)
%     disp("Motor " + string(k) + " Active")
% end

%% Control Loop
while errorEncountered == false
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
    q           = DirectKinematics_V2(a,b,l)   % Measured pose

    % Calculate Structure Matrix
    A = WrenchMatrix_V2(a,b,q);

    % Calculate Errors
    e           = q_d - q;

    Kp = diag([100 1 1]);  % lol

    % Desired wrench (TESTE KONTROLLER)
    w_c = Kp*e;

    [f,w_rizz] = ForceAllocIterativeSlack(A,f_min,f_max,f_ref,f_prev,w_c)

    % Assume ideal world

    T = f*R.*motorsigns*(-1); % fordi motorsigns er for endring i kabelendring og ikke rotasjonsretning på torque


    % Write torque to motor drivers (YOOOO: CHECK POSITIV REGNING)
    for k = 1:length(fieldNames)
        fieldName = fieldNames{k}; % Current field name as a string
        currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
        setMotorTorque(T(k), currentSerialPort)
    end

    % Save previous cable forces
    f_prev = f;

end

end