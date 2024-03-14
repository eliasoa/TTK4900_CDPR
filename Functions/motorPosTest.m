function motorPosTest(ODriveStruct,ODriveEnums)

userInput = input('Enter position setpoint  [-1, 1]: ', 's'); % 's' for string input
x_target = str2double(userInput); % Convert to number
disp(['You entered the setpoint: ', num2str(x_target)]);

escapePressed = false;  % Initialize termination button (Press Esc to )
errorEncountered = false;
fieldNames = fieldnames(ODriveStruct);

h = 0.01;
e_int = 0;
v_prev = [0;0];
R = 0.02;

% Initial Cable Length
l0 = [0.17;0.17];
% Desired Cable Lengths
l2_d = mapfun(x_target, -1, 1, 0.04, 0.30);
l1_d = 0.34 - l2_d;

motorPos = [0;0];
motorVel = [0;0];


for k = 1:length(fieldNames)
    fieldName = fieldNames{k}; % Current field name as a string
    currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
    setAxisState(ODriveEnums.AxisState.AXIS_STATE_CLOSED_LOOP_CONTROL, currentSerialPort)
    disp("Motor " + string(k) + " Active")
end

while escapePressed == false

    charPressed = get(gcf, 'CurrentCharacter');

    % Check which arrow key is pressed
    switch charPressed
        case 27 % Escape key
            escapePressed = true;
    end
    %% If an error occurs, stop motors
    [~, errorsFound, disarmReasonsFound] = getDriverStatus(ODriveStruct, ODriveEnums.Error);
    if errorsFound || disarmReasonsFound
        errorEncountered = true;
        disp("Error occured, stopping motors")
        for k = 1:length(fieldNames)
            fieldName = fieldNames{k}; % Current field name as a string
            currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
            setAxisState(ODriveEnums.AxisState.AXIS_STATE_IDLE, currentSerialPort)
            disp("Motor " + string(k) + " Idle")
        end
        break
    end
    %% Get motor position from encoders
    for k = 1:length(fieldNames)
        currentSerialPort = ODriveStruct.(fieldNames{k}); 
        motorPos(k) = getEncoderPosition(currentSerialPort);
        motorVel(k) = getEncoderVelocity(currentSerialPort);
    end
    
    
    x_r = motorPos(1);
    x_l = motorPos(2);
    x_current = (x_l - x_r) / 2;
    
    % theta1 = motorPos(1);
    % theta2 = motorPos(2);
    % 
    % theta1_dot = motorVel(1);
    % theta2_dot = motorVel(2);
    % 
    % % Estimate cable lengths
    % l1_est = l0(1) + theta1*2*pi*R;
    % l2_est = l0(2) + theta2*2*pi*R;
    % 
    % ldot_est = [theta1_dot*2*pi*R;theta2_dot*2*pi*R];
    % 
    % e1 = l1_d - l1_est;
    % e2 = l2_d - l2_est;
    % 
    % e = [e1;e2]
    % 
    % %% Velocity Controllerererr
    % 
    % K_p = 40;
    % K_d = 2.5;
    % 
    % v1 = K_p * e1 - K_d*ldot_est(1);
    % v2 = K_p * e2 - K_d*ldot_est(2);
    % 
    % v = [v1;v2];
    % 
    % v_tol = 0.05;
    % 
    % for i=1:2
    %     if (v(i) < v_tol) && (v(i) > -v_tol)
    %         v(i) = 0;
    %     end
    % end
    % v
    % 
    % % Write to drivers
    % for k = 1:length(fieldNames)
    %     ODrive = ODriveStruct.(fieldNames{k});
    %     setMotorVelocity(v(k), 0, ODrive)
    % end

    
    %% Tension logic
    % Given values
    t_ref = -0.3;
    % sat_lim = 0.4;
    t_min = -0.1;  % Less tension
    t_max = -0.5; %t_ref - sat_lim;  % More tension

    Kp = 0.5; % Tunable gain
    Kd = 0;
    % Ki = .2;
    
    % Calculate tension adjustment based on error
    % e_int = e_int + e * h
    % 
    % e_tol = 0.1;
    % if e_int >= e_tol
    %     e_int = e_tol
    % elseif e_int <=  -e_tol
    %     e_int = -e_tol
    % end
    e = x_target - x_current

    t = e * Kp;% - e_dot*Kd; %+ e_int * Ki;

    % Calculate tension for each motor, ensuring it stays within limits
    T_right = max(min(t_ref - t, t_min), t_max)
    T_left = max(min(t_ref + t, t_min), t_max)

    % Tension vector to be sent to motors
    T = [T_right; T_left];  
    % disp(["Error: ", e])
    % disp(["E_dot: ", e_dot])
    % disp(["Torque right: ",T_right])
    % disp(["Torque left: ",T_left])
    for k = 1:length(fieldNames)
        ODrive = ODriveStruct.(fieldNames{k});
        setMotorTorque(T(k), ODrive);
    end

    % if abs(e) < 0.01
    %     pause(3);
    %     break;
    % end


end
close all

if~errorEncountered
    for k = 1:length(fieldNames)
        fieldName = fieldNames{k}; % Current field name as a string
        currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
        setAxisState(ODriveEnums.AxisState.AXIS_STATE_IDLE, currentSerialPort)
        disp("Motor " + string(k) + " Idle")
    end
end


end

% This function is a copy of the "map" function used in Arduino code.
% It takes an input value mapped from a min and max possible value and
% scales it to a desired output min and max.
%
% [output] = mapfun(value,fromLow,fromHigh,toLow,toHigh)
% value: can be a scalar entry or a matrix of values
% fromLow: lowest possible input value
% fromHigh: highest possible input value
% toLow: lowest desired output value
% toHigh: highest possible output value
% [output]: output matrix of same size as value
% 
% examples:
% output = mapfun(-.3,-1,1,0,5)
% output =
%
%    1.7500
%
% output = mapfun([-1 1 -.5 .5],-1,1,0,5)
% output =
%
%         0    5.0000    1.2500    3.7500
%
% Created by David Coventry on 1/19/2017

function output = mapfun(value,fromLow,fromHigh,toLow,toHigh)
narginchk(5,5)
nargoutchk(0,1)
output = (value - fromLow) .* (toHigh - toLow) ./ (fromHigh - fromLow) + toLow;
end
