function testing(ODriveStruct, CDPR_Params)
%% Robot parameters
R           = CDPR_Params.Gen_Params.SpoolParams.SPOOL_RADIUS;        % Radius of spool
P           = CDPR_Params.Gen_Params.SpoolParams.PITCH;
r_p         = 0.012; % raduis trinse
a           = CDPR_Params.SGM.FrameAP;                    % Frame Anchor Points
b           = CDPR_Params.SGM.BodyAP.TRAPEZOID;           % Body Anchor Points
motorsigns  = CDPR_Params.Gen_Params.MOTOR_SIGNS;         % Signs determining positive rotational direction
p0          = CDPR_Params.Gen_Params.EncoderOffset;
f_min = 10;
f_max = 80;
f_ref = (f_max +f_min)/2;

f_static = [2; 7; 0; 0];
f_static = [0; 0; 0; 0];
t_static = [0.056; 0.0504; 0.0498; 0.0354];
fvel3 = 0;
fvel = zeros(4,1);

x_diff = 56.40783*1e-3;
y_diff = 371.4759*1e-3;

iter = 200;
count       = 1;
timeVec     = zeros(1,iter);
timeSca     = 0;
%% Preallocation of vector
f_prev      = f_ref*ones(4,1);
l_dot       = zeros(4,1);
T_friction  = zeros(4,1);

% Log vectors
q_log           = zeros(3,iter);
qd_log          = zeros(3,iter);
e_log           = zeros(3,iter);
e_int_log       = zeros(3,iter);
f_log           = zeros(4,iter);
T_friction_log  = zeros(4,iter);
vel_log         = zeros(4,iter);
fvel_log        = zeros(4,iter);

f_uncon_log     = zeros(4,iter);
f_friction_log  = zeros(4,iter);
w_friction_log  = zeros(3,iter);

w_c_log         = zeros(3,iter);


% l0 = [1.2260 1.1833 1.1833 1.2260]';
l0 = [1.2060 1.1820 1.1820 1.2060]';

omega = 2;
Ly = 0.15;
Lx = 0.15;

Kp = diag([200; 175; 5]);
Ki = diag([0;0;0]);
Kd = diag([10 10 0]);
e_int   = zeros(3,1);

fieldNames = fieldnames(ODriveStruct);

%% Control Loop
s_start = tic;
while count <= iter
    t_loop = tic;
    [pos, vel] = TEST_getEncoderReading(ODriveStruct);
    pos_n = EncoderOffset(pos, p0);
    pos_rad = pos_n*2*pi;
    l_enc = pos_rad*R.*motorsigns;
    l_q = l0 + l_enc;
    w_d = pos_n*P;
    l_spool_to_pulley = sqrt(sqrt(w_d.^2+y_diff^2).^2 + x_diff^2);
    l_q_fk = l_q - l_spool_to_pulley;

    q_0 = init_fk_estimate(a,b,l_q_fk);
    q = p_forward_kinematics(a,b,l_q_fk,q_0,r_p);

    time = toc(s_start);
    q_d         = [Lx*sin(omega*time);Ly*cos(omega*time)-Ly;0];
    q_d_dot     = [-Lx*cos(omega*time); Ly*sin(omega*time); 0];

    % Calculate Structure Matrix
    [~,betar]   = p_inverse_kinematics(a,b,q, r_p);
    A_t         = structure_matrix(a,b,q,r_p,betar);
    A_pseudo    = pinv(A_t');
    A_t_pinv    = pinv(A_t);

    q_dot = -A_pseudo*l_dot;

    % Calculate Errors
    e           = q_d - q;
    e_dot       = q_d_dot - q_dot;

    % Desired wrench (TESTE KONTROLLER)
    w_cP = Kp*e;
    w_cI = Ki*e_int;
    w_cD = Kd*e_dot;
    w_c =  w_cP + w_cI + w_cD;
    w_c = [15;0;0];
    
    f_uncon = A_t_pinv*w_c;

    % Force Allocation
    [f,~,flag] = ForceAllocIterativeSlack(A_t',f_min,f_max,f_ref,f_prev,w_c);
    % fvel3 = 0.9*fvel3 + 0.1*vel(3);
    fvel = 0.6*fvel + 0.4*vel;
    for i = 1:4
        % if i==4
            % T_friction(i) = FrictionModel(i-1,fvel3);
        % else
            T_friction(i) = FrictionModel(i-1,fvel(i));
            T_friction_signed(i) = T_friction(i)*motorsigns(i);
        % end
        
    end
    f_friction = T_friction./R;
    f_friction_signed = T_friction_signed./R;
    w_friction = -A_t*f_friction;
  
    T_s = ...
    sign(A_t_pinv*w_c).*(~(ones(4,1)&fix(fvel*10^3)/10^3)...
    &any(fix(e*10^3)/10^3)).*t_static
    % fix(vel*10^3)/10^3
    % any(fix(e*10^3)/10^3)
    % (ones(4,1)&fix(vel*10^1)/10^1)
    % T = ((f)*R+T_friction+T_s).*motorsigns*(-1);
    % T = ((f)*R).*motorsigns*(-1);
    T = (f*R).*motorsigns*(-1);

    % Write torque to motor drivers (YOOOO: CHECK POSITIV REGNING)
    for k = 1:length(fieldNames)
        fieldName = fieldNames{k}; % Current field name as a string
        currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
        setMotorTorque(T(k), currentSerialPort);
    end

    % Save previous cable forces
    if ~flag
        f_prev = f;
    end

    % Logging
    q_log(:,count)      = q;
    qd_log(:,count)     = q_d;
    e_log(:,count)      = e;
    e_int_log(:,count)  = e_int;
    f_log(:,count)      = f;
    T_friction_log(:,count) = T_friction;
    vel_log(:,count)    = vel;    
    fvel_log(:,count)   = fvel;    

    f_uncon_log(:,count) = f_uncon;
    f_friction_log(:,count) = f_friction;
    f_friction_signed_log(:,count) = f_friction_signed;
    w_friction_log(:, count) = w_friction;
    
    w_c_log(:,count) = w_c;

    total_time(count)   = toc(t_loop);
    timeSca = timeSca + total_time(count);
    timeVec(count) = timeSca;

    count               = count + 1;
end

%% Save logged data
save("PlotData/PoseData.mat", "q_log","qd_log","e_log","e_int_log","f_log","T_friction_log","f_uncon_log","f_friction_log","f_friction_signed_log", "w_friction_log", "w_c_log", "timeVec", "vel_log", "fvel_log");
% Plot of total time and Hertz
% figure(1)
% subplot(2,1,1)
% hold on
% plot(total_time)
% ylabel({'Seconds'});
% xlabel({'Iteration number'});
% title({'Loop time'});
% hold off
% subplot(2,1,2)
% hold on
% plot(1./total_time)
% ylabel({'Hz'});
% xlabel({'Iteration number'});
% title({'Loop frequency'});
% hold off
T = [0.2;-0.2;0.2;-0.2];
for k = 1:length(fieldNames)
    fieldName = fieldNames{k}; % Current field name as a string
    currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
    setMotorTorque(T(k), currentSerialPort);
end

end