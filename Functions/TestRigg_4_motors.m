function TestRigg_4_motors(ODriveStruct, ODriveEnums, CDPR_Params)

%% Parameters
R           = CDPR_Params.Gen_Params.SPOOL_RADIUS;        % Radius of spool
a           = CDPR_Params.SGM.FrameAP;                    % Frame Anchor Points
b           = CDPR_Params.SGM.BodyAP.RECTANGLE;           % Body Anchor Points
motorsigns  = CDPR_Params.Gen_Params.MOTOR_SIGNS;         % Signs determining positive rotational direction
m_p         = CDPR_Params.Gen_Params.Platform_mass;       % Mass of MP
f_min = 0.1/R;
f_max = 1.5/R;
f_ref = 0.5/R;

%% Initialize variables
x   = 0.08;                        % Desired x-position
y   = 0;                            % Desired y-position
phi = 0;                            % Desired phi-angle [radians]

q_d         = [x;y;phi];            % Desired pose
q_d_dot     = [0;0;0];              % Desired velocity

escapePressed = false;              % Initialize termination button (Press Esc to )
errorEncountered = false;           % Initialize error counter bit

posIncrement    = 0.5;              % Position Increment each arrow click
angleIncrement  = 2;                % Angle increment each arrow click

% Initialize full states
q0          = [0;0;0];              % Initial Pose
dq0         = [0;0;0];              % Initial Velocity
l0          = [0.2938;0.3132;0.3124;0.2946];
[~,l0,~] = CDPR_InverseKinematics_V2([0;0;0], a, b)


% s           = [q0;dq0;zeros(6,1)];    % State Vector
% e           = zeros(6,1);             % Memory Allocation for pose error
e_int       = zeros(3,1);               % Memory Allocation for integral of error
f_prev      = [0.39/R;0.39/R;0.39/R;0.39/R];           % Memory Allocation for prev cable forces

l           = zeros(4,1);               % Memory Allocation for cable lengths
l_dot       = zeros(4,1);               % Memory Allocation for cable velocities

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
    

    l;
    l0;

    % Calculate Structure Matrix
    A = WrenchMatrix_V2(a,b,q);
    % A_pseudo = pinv(A);

    % Calculate current velocity of the platform
    % % q_dot       = -A_pseudo*l_dot             % Estimated velocity (Not currently in use)

    % Calculate Errors
    % q(3) = 0;
    e           = q_d - q
    % e_dot       = q_d_dot - q_dot
    % e(3) = 0; %Tihi hax
    % e

    Kp = diag([80 80 1]);  % lol
    Kd = diag([25 25 0]);
    Ki = diag([10 10 1]);
    % K_d = 1;

    % Desired wrench (TESTE KONTROLLER)
    w_c = Kp*e %+ Kd*e_dot

    % [f, flag] = Optimal_ForceDistributions(A,w_c,m_p,f_min,f_max,f_ref, f_prev)


    % Assume ideal world

    T = f*R.*motorsigns*(-1) % fordi motorsigns er for endring i kabelendring og ikke rotasjonsretning på torque
    

    % Write torque to motor drivers (YOOOO: CHECK POSITIV REGNING)
    for k = 1:length(fieldNames)
        fieldName = fieldNames{k}; % Current field name as a string
        currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
        setMotorTorque(T(k), currentSerialPort)
    end

    % Save previous cable forces
    f_prev = f;

    % Update Integral effect (later)
    % e_int = e_int + e*


end

end

function escapePressed = KeyPressFcn(~, event)
        fprintf('Key pressed: %s\n', event.Key);
        if strcmp(event.Key, 'escape')
            escapePressed = true;
        else 
            escapePressed = false;
        end
    end