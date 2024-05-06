% function test(ODriveStruct, ODriveEnums, CDPR_Params)

% userInput = input("Ensure that the MP is fastened at the origin with the drill bit. Type y when done: ", 's');
% if userInput == 'y'
%     disp("Setting homing tension")
%     T = [0.2; -0.2; 0.2; -0.2];
%     fieldNames = fieldnames(ODriveStruct);
%     for k = 1:length(fieldNames)
%         fieldName = fieldNames{k}; % Current field name as a string
%         currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
%         setAxisState(ODriveEnums.AxisState.AXIS_STATE_CLOSED_LOOP_CONTROL, currentSerialPort)
%         disp("Motor " + string(k) + " Active")
%         pause(0.01)
%         setMotorTorque(T(k), currentSerialPort)
%         % pause(0.1)
%     end
% end


%% Parameters
R           = CDPR_Params.Gen_Params.SpoolParams.SPOOL_RADIUS;        % Radius of spool
P           = CDPR_Params.Gen_Params.SpoolParams.PITCH;
r_p         = 0.012; % raduis trinse
a           = CDPR_Params.SGM.FrameAP;                    % Frame Anchor Points
b           = CDPR_Params.SGM.BodyAP.TRAPEZOID;           % Body Anchor Points
motorsigns  = CDPR_Params.Gen_Params.MOTOR_SIGNS;         % Signs determining positive rotational direction
% m_p         = CDPR_Params.Gen_Params.Platform_mass;       % Mass of MP
f_min = 15;
f_max = 80;
f_ref = (f_max +f_min)/2;

%Solve shit
x_diff = 56.40783*1e-3;
y_diff = 371.4759*1e-3;

f_prev      = f_ref*ones(4,1);

betae = 0.15;

K = 638; % [N/m] Spring constant for cable model, based on Youngs modulus of Dynema SK78of 109 - 132 GPa approx

fieldNames = fieldnames(ODriveStruct);

% l0 = [1.2260 1.1833 1.1833 1.2260]';
% l0 = [1.21 1.120 1.120 1.21]';
l0 = [1.2060 1.1820 1.1820 1.2060]';

%% Get current position and cable lengths at that position
[pos, ~] = TEST_getEncoderReading(ODriveStruct);
pos_n = EncoderOffset(pos, p0);

pos_rad = pos_n*2*pi;
l_enc = pos_rad*R.*motorsigns;
l_q = l0 + l_enc;
w_d = pos_n*P;
l_spool_to_pulley = sqrt(sqrt(w_d.^2+y_diff^2).^2 + x_diff^2);
l_q_fk = l_q - l_spool_to_pulley

q_0 = init_fk_estimate(a,b,l_q_fk);
q = p_forward_kinematics(a,b,l_q_fk,q_0,r_p);
% q = [0 0 0]';

%% Calulate cable length in desired pose
% Desired pose
x = 0.1;
y = 0;
phi = 0;

qd = [x;y;phi]

% Cable lengths at desired pose
[l_qd_ik,betar_qd] = p_inverse_kinematics(a, b, qd, r_p)
l_qd_ik = l_qd_ik+r_p*(betar_qd-betae)

%% Calculate the needed change in additional position to get the desired tension @ qd
% Calculate Structure Matrix
[~,betar]   = p_inverse_kinematics(a,b,qd, r_p);
A_t         = structure_matrix(a,b,qd,r_p,betar);
% No desired wrench in desired position
w_c = [0; 0; 0];
% Calculate cable forces
[f,~,flag] = ForceAllocIterativeSlack(A_t',f_min,f_max,f_ref,f_prev,w_c);

% Using Hookes law to calculate delta_x
dx = f./K;

%% Calculate needed change in cable lenghts and the corresponding change in motor position
% Needed change in cable length
dl = l_qd_ik - l_q_fk;
% Needed motor position change
enc_pos = dl/(2*pi);


% Calculate direction motors must turn
motor_pos_change = (enc_pos).* motorsigns;
motor_pos = pos + motor_pos_change;

%% TODO: For testing
%% Write moto_pos_change to motors
v_lim = 10; %?
t_lim = 10; %?
for k = 1:length(fieldNames)
    fieldName = fieldNames{k}; % Current field name as a string
    currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
    % setMotorPosition(motor_pos(k),v_lim, t_lim,currentSerialPort);
end

%% Write additional change due to tension to motor
% Add extra change to do tension
motor_pos_change = motor_pos + dx.*motorsigns;
motor_pos = pos + motor_pos_change;
pause(1); % For visuallity ;)

for k = 1:length(fieldNames)
    fieldName = fieldNames{k}; % Current field name as a string
    currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
    % setMotorPosition(motor_pos(k),v_lim, t_lim,currentSerialPort);
end

%% Check if wanted tension is held at qd
T = f*R;
T_real = zeros(4,1);
for k = 1:length(fieldNames)
    fieldName = fieldNames{k}; 
    currentSerialPort = ODriveStruct.(fieldName);
    T_real(k) = getTorqueEstimate(currentSerialPort); % Noisy as shit
end
T_error = T - T_real


% p0 = CDPR_Params.Gen_Params.EncoderOffset
% while true
%
% [pos, ~] = TEST_getEncoderReading(ODriveStruct);
%
% pos_n = EncoderOffset(pos, p0);
% p0
% pos-pos_n
%     for k = 1:4
%         pos_rad(k) = pos_n(k)*2*pi;
%         l_enc(k) = pos_rad(k)*R*motorsigns(k);
%         l(k) = l0(k) + l_enc(k);
%         w_d(k) = pos_n(k)*P;
%         l_fk(k) = l(k) - sqrt(sqrt(w_d(k)^2+y_diff^2)^2 + x_diff^2);
%
%         % Cable Velocity
%         % theta_m_dot(k)  = vel(k)*2*pi*motorsigns(k);
%         % l_dot(k)        = theta_m_dot(k)*R;
%     end
%
%
%     %% Controller
%     % Calculate current and desired pose of the platform
%     q_0 = init_fk_estimate(a,b,l_fk);
%     q = p_forward_kinematics(a,b,l_fk,q_0,r_p);
%     q(1:2)
%     rad2deg(q(3))
%     [lf,~] = p_inverse_kinematics(a,b,q, r_p);
% end
