%% Preamble
close
clear
%% Add folders with functions to path
% Name of the folder to add
folderName = 'Functions';

% Get the current working directory
currentDir = pwd;

% Construct the full path to the folder
folderPath = fullfile(currentDir, folderName);

% Add the folder to the MATLAB path
addpath(folderPath);

%% Set up ASCII communication
baudrate = 115200;
timeout = 1;
ODriveStruct = initSerialPorts(baudrate, timeout);

%% Generate structs with ODrive modes and error codes (enum from Arduino)
% Each struct has to be passed as an argument if they need to be used in a
% function

init_ODriveEnums;
init_CDPR_Params;

%% How to access each driver in the driver struct
% Assuming you have the structure serialPortsStruct with dynamic field names

% % Get all field names in the structure
% fieldNames = fieldnames(ODriveStruct);
%
% % Iterate over each field in the structure
% for k = 1:length(fieldNames)
%     fieldName = fieldNames{k}; % Current field name as a string
%     currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
%
%     % Now you can use currentSerialPort as needed
%     setAxisState(1,currentSerialPort);
% end
%% Main program
while true
    userInput = input('Enter a number or type "exit" to stop: ', 's'); % 's' for string input
    % userInput = '8';
    if strcmp(userInput, 'exit')
        clc
        disp('Exiting...');
        break; % Exit the loop
    elseif all(isstrprop(userInput, 'digit')) % Check if all characters are digits
        number = str2double(userInput); % Convert to number
        disp(['You entered the number: ', num2str(number)]);

        % Switch-case structure to handle different numeric inputs
        switch number

            case 1
                clc
                disp("Move with cursor mode")
                disp("Press Esc to exit")
                % arrowKey_manual_control(ODriveStruct, ODriveEnums, CDPR_Params);
                arrowKeyDebugger(CDPR_Params)
                clc

            case 2
                clc
                disp("Set homing tension")
                % isError = checkForErrors(ODriveStruct,ODriveEnums);
                % if isError
                %     disp('Errors, please clear them with 4')
                %     return
                % end
                userInput = input("Ensure that the MP is fastened at the origin with the drill bit. Type y when done: ", 's');
                if userInput == 'y'
                    disp("Setting homing tension")
                    T = [0.39; -0.39; 0.39; -0.39];
                    fieldNames = fieldnames(ODriveStruct);
                    for k = 1:length(fieldNames)
                        fieldName = fieldNames{k}; % Current field name as a string
                        currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
                        setAxisState(ODriveEnums.AxisState.AXIS_STATE_CLOSED_LOOP_CONTROL, currentSerialPort)
                        disp("Motor " + string(k) + " Active")
                        pause(0.01)
                        setMotorTorque(T(k), currentSerialPort)
                        % pause(0.1)
                    end
                else
                    disp("Insert homing plug and try again xddddd")
                end

            case 3
                disp("Set home position")
                % Set home position
                fieldNames = fieldnames(ODriveStruct);
                for k = 1:length(fieldNames)
                    fieldName = fieldNames{k}; % Current field name as a string
                    currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
                    setEncoderPositions(currentSerialPort);
                end


            case 4
                clc
                disp("Clear errors")
                errorCheckAndHandling(ODriveStruct, ODriveError);

            case 5
                clc
                disp("Testing: 4 motors")
                % a = CDPR_Params.SGM.FrameAP;
                % b = CDPR_Params.SGM.BodyAP.TRAPEZOID;

                % [L0,l0, A_transposed] = CDPR_InverseKinematics_V2([0;0;0], a, b)
                % l0 = [0.8228;0.7798;0.7798;0.8228];
                % q = DirectKinematics_V2(a,b,l0)
                TestRigg_4_motors(ODriveStruct, ODriveEnums, CDPR_Params);

            case 6
                disp("Enable enable_dc_bus_voltage_feedback on all ODrives")
                % calibrateMotor(ODriveStruct.ODrive0,ODriveEnums)
                fieldNames = fieldnames(ODriveStruct);
                for k = 1:length(fieldNames)
                    fieldName = fieldNames{k}; % Current field name as a string
                    currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field name
                    enableBrakeResistorVoltageFeedback(currentSerialPort);
                end

            case 7
                disp("Check workspace")
                % motorPosTest(ODriveStruct, ODriveEnums);
                phi_0 = 0.09;
                R           = CDPR_Params.Gen_Params.SpoolParams.SPOOL_RADIUS;        % Radius of spool
                a           = CDPR_Params.SGM.FrameAP;                    % Frame Anchor Points
                b           = CDPR_Params.SGM.BodyAP.TRAPEZOID;           % Body Anchor Points
                motorsigns  = CDPR_Params.Gen_Params.MOTOR_SIGNS;         % Signs determining positive rotational direction
                m_p         = CDPR_Params.Gen_Params.MASS_PLATFORM;       % Mass of MP
                f_min = 0.2/R;
                f_max = 0.6/R;
                f_ref = 0.4/R;
                w = [0 -m_p*.81 0]';
                resolution = 100;
                color = 'red';
                TranslationWorkspace_V2(phi_0,a,b,m_p, f_min,f_max, f_ref, w, resolution, color)

            case 8
                isError = checkForErrors(ODriveStruct,ODriveEnums);
                if isError
                    disp('Errors, please clear them with 4')
                    return
                end
                force_struct = load("sine_forces_2.mat");
                forces = force_struct.f;
                mpGoesSpin(forces, ODriveEnums,ODriveStruct);
            case 9
                T = [0; 0; 0; 0];
                fieldNames = fieldnames(ODriveStruct);
                for k = 1:length(fieldNames)
                    fieldName = fieldNames{k}; % Current field name as a string
                    currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
                    setAxisState(ODriveEnums.AxisState.AXIS_STATE_CLOSED_LOOP_CONTROL, currentSerialPort)
                    disp("Motor " + string(k) + " Active")
                    pause(0.01)
                    setMotorTorque(T(k), currentSerialPort)
                    setAxisState(ODriveEnums.AxisState.AXIS_STATE_IDLE, currentSerialPort)
                    % pause(0.1)
                end


            otherwise
                disp('Input number does not match any function.');
                clc
        end
    else
        disp('Invalid input. Please enter a number or "exit".');
    end
end
