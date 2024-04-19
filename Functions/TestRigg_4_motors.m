function TestRigg_4_motors(ODriveStruct, ODriveEnums, CDPR_Params)

%% Parameters
R           = CDPR_Params.Gen_Params.SpoolParams.SPOOL_RADIUS;        % Radius of spool
P           = CDPR_Params.Gen_Params.SpoolParams.PITCH;
r_p         = 0.012; % raduis trinse
a           = CDPR_Params.SGM.FrameAP;                    % Frame Anchor Points
b           = CDPR_Params.SGM.BodyAP.TRAPEZOID;           % Body Anchor Points
motorsigns  = CDPR_Params.Gen_Params.MOTOR_SIGNS;         % Signs determining positive rotational direction
% m_p         = CDPR_Params.Gen_Params.Platform_mass;       % Mass of MP
f_min = 5;
f_max = 60;
f_ref = 25;

% Solve shit
x_diff = 56.40783*1e-3;
y_diff = 371.4759*1e-3;

% Torque Offsets
offset2 = [0.13; 0.11; 0.1; 0.2];


%% Initialize variables
x   = -0.25;                        % Desired x-position
y   = 0;                          % Desired y-position
phi = 0;                            % Desired phi-angle [radians]

q_d         = [x;y;phi];            % Desired pose
q           = [0;0;0];

errorEncountered = false;           % Initialize error counter bit


l0 = [1.2260 1.1833 1.1833 1.2260]';

% [l,betar] = p_inverse_kinematics(a,b,q, r_p)

f_prev      = f_ref*ones(4,1);   % Memory Allocation for prev cable forces

l           = zeros(4,1);                   % Memory Allocation for cable lengths
l_fk           = zeros(4,1);                   % Memory Allocation for cable lengths
l_enc = zeros(4,1);
w_d = zeros(4,1);
% l_dot       = zeros(4,1);                   % Memory Allocation for cable velocities

% Sine testing - ellipse
omega = 0.5;
h_test = 0.2;
Ly = 0.1;
Lx = 0.2;
time_test = 0;

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
    tic
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
        % vel = getEncoderVelocity(currentSerialPort);                 % Get angular position and velocity from encoder
        pos_rad(k) = pos*2*pi;
        l_enc(k) = pos_rad(k)*R*motorsigns(k);



        l(k) = l0(k) + l_enc(k);
        w_d(k) = pos*P;

        l_fk(k) = l(k) - sqrt(sqrt(w_d(k)^2+y_diff^2)^2 + x_diff^2);

        % l(k)        = encoder2cableLen(pos,l(k),R, motorsigns(k)); % Estimate cable length
        % l_dot(k)    = encoder2cableVel(vel, CDPR_Params, motorsigns(k));        % Estimated Rate of change of cable
    end
    % l_fk

    %% Controller
    % Calculate current and desired pose of the platform

    % q           = DirectKinematics_V2(a,b,l)   % Measured pose
    q_0 = init_fk_estimate(a,b,l_fk)
    q = p_forward_kinematics(a,b,l_fk,q_0,r_p)

    % % Test sine
    % q_d         = [Lx*sin(omega*time_test);Ly*cos(omega*time_test);phi];            % Desired pose
    
    rad2deg(q(3))

    % Calculate Structure Matrix
    [~,betar] = p_inverse_kinematics(a,b,q, r_p);
    A_t = structure_matrix(a,b,q,r_p,betar);

    % Calculate Errors
    e           = q_d - q;

    Kp = diag([225 215 1]);%diag([250 100 1]);  % lol

    % Desired wrench (TESTE KONTROLLER)
    w_c =  Kp*e;

    [f,w_rizz,flag] = ForceAllocIterativeSlack(A_t',f_min,f_max,f_ref,f_prev,w_c)

    % Assume ideal world

    T = f*R.*motorsigns*(-1); % fordi motorsigns er for endring i kabelendring og ikke rotasjonsretning på torque

    T = T + sign(T).*offset2


    % Write torque to motor drivers (YOOOO: CHECK POSITIV REGNING)
    for k = 1:length(fieldNames)
        fieldName = fieldNames{k}; % Current field name as a string
        currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
        % setMotorTorque(T(k), currentSerialPort)
    end

    % Save previous cable forces
    if ~flag
        f_prev = f;
    end

    % time_test = time_test + h_test;

    toc
end

end