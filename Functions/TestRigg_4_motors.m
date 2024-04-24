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
T_s = [0.2;0.15;0.14;0.12]; % Starting torque for motors (from non-cabling testing)
T_s = [0.2;0.2;0.2;0.2]; % Starting torque for motors
T_s = [0.3;0.3;0.2;0.2]; % Starting torque for motors
T_s = 0.05*ones(4,1);%[0.1;0.1;0.1;0.1]; % Starting torque for motors
precV = 1; % Precision for evaluating velocity
precT = 1; % Precision for evaluating torque


%% Initialize variables
x   = -0.25;                        % Desired x-position
y   = 0;                          % Desired y-position
phi = 0;                            % Desired phi-angle [radians]

q_d         = [x;y;phi];            % Desired pose
q           = [0;0;0];

l0 = [1.2260 1.1833 1.1833 1.2260]';

errorEncountered = false;           % Initialize error counter bit

%% Control
Kp      = diag([200 200 5]);%diag([250 100 1]);  % lol
Ki      = diag([0 0 0]); 
e_int   = zeros(3,1);
% tolInt

%% Preallocation of vectors
f_prev      = f_ref*ones(4,1);  % Memory Allocation for prev cable forces

l           = zeros(4,1);       % Memory Allocation for cable lengths
l_fk        = zeros(4,1);       % Memory Allocation for cable lengths
l_enc       = zeros(4,1);
w_d         = zeros(4,1);
l_dot       = zeros(4,1);       % Memory Allocation for cable velocities
pos_rad     = zeros(4,1);
vel         = zeros(4,1);

%% Logging 
iter = 100;
total_time  = zeros(1,iter);
count       = 1;
tIter = linspace(1,iter,1);

q_log       = zeros(3,iter);
qd_log      = zeros(3,iter);
e_log       = zeros(3,iter);    

%% Sine testing - ellipse
omega = 3;
h_test = 0.2;
Ly = 0.2;
Lx = 0.2;
time_test = 0;
h_sample = 0.06;




fieldNames = fieldnames(ODriveStruct);
%% Control Loop
s_start = tic;
while errorEncountered == false && count <= iter
t_loop = tic;
    %% Estimate Cable Lengths
    % tic
    [pos, vel] = TEST_getEncoderReading(ODriveStruct);
    for k = 1:4      
        pos_rad(k) = pos(k)*2*pi;
        l_enc(k) = pos_rad(k)*R*motorsigns(k);

        l(k) = l0(k) + l_enc(k);
        w_d(k) = pos(k)*P;

        l_fk(k) = l(k) - sqrt(sqrt(w_d(k)^2+y_diff^2)^2 + x_diff^2);
        % t_solveMatte = toc
        % l(k)        = encoder2cableLen(pos,l(k),R, motorsigns(k)); % Estimate cable length
        % l_dot(k)    = encoder2cableVel(vel, CDPR_Params, motorsigns(k));        % Estimated Rate of change of cable
    end
    % t_getCables = toc;

    %% Controller
    % Calculate current and desired pose of the platform

    % q           = DirectKinematics_V2(a,b,l)   % Measured pose
    % tic
    q_0 = init_fk_estimate(a,b,l_fk);
    q = p_forward_kinematics(a,b,l_fk,q_0,r_p);
    % t_forward = toc

    % % Test sine
    bla = toc(s_start);
    q_d         = [Lx*sin(omega*bla);Ly*cos(omega*bla);phi];            % Desired pose
    % q_d         = [0;Ly*cos(omega*bla);phi];
    % q_d         = [Lx*sin(omega*bla);0;phi];
    % q_d = [0.2; 0.2; 0];
    % rad2deg(q(3));

    % Calculate Structure Matrix
    % tic
    [~,betar] = p_inverse_kinematics(a,b,q, r_p);
    % t_inverse = toc
    
    % tic
    A_t = structure_matrix(a,b,q,r_p,betar);
    % t_struct_mat = toc
    % Calculate Errors
    e           = q_d - q;
    
    
    
    % Desired wrench (TESTE KONTROLLER)
    w_c =  Kp*e + Ki*e_int;
    % tic
    [f,w_rizz,flag] = ForceAllocIterativeSlack(A_t',f_min,f_max,f_ref,f_prev,w_c);
    % t_force = toc
    % Assume ideal world
      
    

    T = f*R.*motorsigns*(-1); % fordi motorsigns er for endring i kabelendring og ikke rotasjonsretning på torque
    % tic
    % offset = ~(ones(4,1)&(fix(vel*10^precV)/10^precV|~(fix(T*10^precT)/10^precT))).*sign(T).*T_s;
    % t_sol = toc
    % T = T + offset;


    % Write torque to motor drivers (YOOOO: CHECK POSITIV REGNING)
    % tic
    for k = 1:length(fieldNames)
        fieldName = fieldNames{k}; % Current field name as a string
        currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
        setMotorTorque(T(k), currentSerialPort);
    end
    % t_writeTorque = toc
    % Save previous cable forces
    if ~flag
        f_prev = f;
    end

    % time_test = time_test + h_test;

    % Update Integral Error
    e_int = e_int + e*toc(t_loop);
    
    % Logging
    total_time(count)   = toc(t_loop);
    q_log(:,count)      = q;
    qd_log(:,count)     = q_d;
    e_log(:,count)      = e; 
    count               = count + 1;
end
%% Save logged data
save("PlotData/PoseData.mat", "q_log","qd_log","e_log");
%% Plot of total time and Hertz
figure(1)
subplot(2,1,1)
hold on
plot(total_time)
ylabel({'Seconds'});
xlabel({'Iteration number'});
title({'Loop time'});
hold off
subplot(2,1,2)
hold on
plot(1./total_time)
ylabel({'Hz'});
xlabel({'Iteration number'});
title({'Loop frequency'});
hold off



end