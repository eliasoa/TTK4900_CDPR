% FROM CHATGPT og mæggærn
function arrowKey_manual_control(ODriveStruct, ODriveEnums, CDPR_Params)

% Extract SerialPorts

% Get all field names in the ODrive struct
fieldNames = fieldnames(ODriveStruct);

% Initialize variables
x   = 0;                % Desired x-position
y   = 0;                % Desired y-position
phi = 0;                % Desired phi-angle [radians]
escapePressed = false;  % Initialize termination button (Press Esc to )

L               = 2.0;      % Length of platform
posIncrement    = 0.5;      % Position Increment each arrow click
angleIncrement  = 2;        % Angle increment each arrow click
xRange          = 20;       % Width of frame
yRange          = 20;       % Height of frame

% Initialize full states
s           = [q0;dq0;zeros(6,1)];          % States        ( MÅ HENTE INITIAL CONDITIONS FRA ET STED, CDPR_PARAMS???  
e           = zeros(6,1);
e_int       = zeros(6,1);
q_dot_d     = zeros(3,1);    % OBS: DENNE MÅ SIKKERT ENDRES PÅ

% Plot initial point
figure;
plot(x, y, 'o');
hold on;
plot([x x+0.5*L*cosd(phi)], [y y+0.5*L*sind(phi)])
plot([x x-0.5*L*cosd(phi)], [y y-0.5*L*sind(phi)])
hold off;

title('Arrow Key Demo');
xlim([-xRange/2, xRange/2])
ylim([-yRange/2, yRange/2])
grid on;

% Get initial encoder positions
encoder_zeros = zeros(4,1);

for k = 1:length(fieldNames)
    fieldName = fieldNames{k}; % Current field name as a string
    currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names

    [pos,~] = getEncoderFeedback(currentSerialPort);
    encoder_zeros(k) = pos;

    setAxisState(ODriveEnums.AxisState.AXIS_STATE_CLOSED_LOOP_CONTROL, currentSerialPort)
    disp("Motor " + string(k) + " Active")
end

%%%%%%%%%%%%%%%%%%%%%%%%
% MANGLER L0, R, a, b
%%%%%%%%%%%%%%%%%%%%%%%%

l0 = [1];
R = CDPR_Params.Gen_Params.SPOOL_RADIUS;    % Extract spool radius from struct
a = CDPR_Params.SGM.FrameAP;                % Extract Frame Anchor points from struct
b = CDPR_Params.SGM.BodyAP.RECTANGLE;       % Extract Body Anchor points from struct
h = CDPR_Params.Gen_Params.SAMPLING_TIME;

l       = zeros(4,1);                                                         % Allocate memory for estimated cable lengths
l_dot   = zeros(4,1);                                                         % Alloctae memory for estimated rate of change of cable

% Wait for arrow key press
while escapePressed == false
    key = waitforbuttonpress;

    % Check if the key press is valid
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

        % Update the plot
        clf;
        plot(x, y, 'o');
        hold on;
        plot([x x+0.5*L*cos(phi)], [y y+0.5*L*sin(phi)])
        plot([x x-0.5*L*cos(phi)], [y y-0.5*L*sin(phi)])
        hold off;
        title('Arrow Key Demo');
        xlim([-xRange/2, xRange/2])
        ylim([-yRange/2, yRange/2])
        grid on;

        
       
        % Estimate Cable Lengths
       
        for k = 1:length(fieldNames)
            fieldName = fieldNames{k}; % Current field name as a string
            currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names

            % flush(currentSerialPort)
            [pos, vel] = getEncoderFeedback(currentSerialPort);                        % Get angular position of encoder

            if (-1)^(k) == -1                                                   % Determine motorsign (Even or odd, can change this)!!!!!!!!!!!!!!!!!!!
                motorsign = 0;
            else
                motorsign = 1;
            end

            l(k)        = encoder2cableLen(encoder_zeros(k), pos,l0(k),R, motorsign);       % Estimate cable length
            l_dot(k)    = encoder2cableVel(vel, CDPR_Params);                               % Estimated Rate of change of cable
        end

        %% Robot Control

        % Calculate current and desired pose of the platform
        q           = DirectKinematics_V2(a,b,l);   % Estimated pose
        q_d         = [x;y;phi];                    % Desired pose

        % Calculate Structure Matrix
        A = WrenchMatrix_V2(a,b,q);
        A_pseudo = pinv(A);

        % Calculate current velocity of the platform 
        q_dot       = -A_pseudo*l_dot;              % Estimated velocity
        
        % Define Full States
        s           = [q;q_dot;];                   % Current state
        s_d         = [q_d;zeros(3,1)];             % Desired state
        
        % % INSERT MOTOR CONTROLLER (CONTROLLER IS NOT COMPLETED)
        [t1,t2,t3,t4,f]    = CDPR_controller(s, s_d, CDPR_Params);
        T                  = [t1,t2,t3,t4];

        % Write torque to motor drivers
        for k = 1:length(fieldNames)
            fieldName = fieldNames{k}; % Current field name as a string
            currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
            setMotorTorque(T(k), currentSerialPort)
        end

        % Calculate Resultant Forces
        % wr = calculate_resultant_forces(f, b, q);

        % Forward Euler 
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
        % (KAN HENDE VI MÅ ENDRE DENNE, HØR MED SØLVE)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

        % s_dot = CDPR_Params.SystemMatrices.A_c_aug*s + CDPR_Params.SystemMatrices.B_c_aug*wr + CDPR_Params.SystemMatrices.d_c_aug + CDPR_Params.SystemMatrices.zeroI*s_d;

        % Update integral states
        e       = s_d - q;
        e_int   = e_int + e*CDPR_Params.Gen_Params.SAMPLING_TIME; 
       
    end
end
% Terminate arrowkeyDemo
close all
for k = 1:length(fieldNames)
    fieldName = fieldNames{k}; % Current field name as a string
    currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
    setAxisState(ODriveEnums.AxisState.AXIS_STATE_IDLE, currentSerialPort)
    disp("Motor " + string(k) + " Idle")
end
end