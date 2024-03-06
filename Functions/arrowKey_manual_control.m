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
R = 0.02;
a = CDPR_Params.SGM.FrameAP;                % Extract Frame Anchor points from struct
b = CDPR_Params.SGM.BodyAP.RECTANGLE;       % Extract Body Anchor points from struct

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

        % Robot Control
        q_d = [x;y;phi];    % Desired state

        % Estimate Cable Lengths
        l = zeros(1,1);                                                         % Allocate memory for estimated cable lengths
        for k = 1:length(fieldNames)
            fieldName = fieldNames{k}; % Current field name as a string
            currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names

            % flush(currentSerialPort)
            [pos, ~] = getEncoderFeedback(currentSerialPort);                        % Get angular position of encoder

            if (-1)^(k) == -1                                                   % Determine motorsign (Even or odd, can change this)
                motorsign = 0;
            else
                motorsign = 1;
            end
            l(k) = encoder2cableLen(encoder_zeros(k), pos,l0(k),R, motorsign);  % Estimate cable length
        end

        % INSERT DIRECT KINEMATICS FROM ESTIMATED CABLE LENGTHS
        % q = DirectKinematics_V2(a,b,l);
        q = [l-l0;0;0];

        % % INSERT MOTOR CONTROLLER (CONTROLLER IS NOT COMPLETED)
        [t1,t2,t3,t4] = CDPR_controller(q_d, q, CDPR_Params);
        T = [t1,t2,t3,t4];

        % INSERT WRITING TORQUE TO DRIVER (NOT DONE, NEED TO ADD DRIVER COMS TO INPUT)
        for k = 1:length(fieldNames)
            fieldName = fieldNames{k}; % Current field name as a string
            currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
            setMotorTorque(T(k), currentSerialPort)
        end

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