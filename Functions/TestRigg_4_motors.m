function TestRigg_4_motors(ODriveStruct, ODriveEnums, CDPR_Params)

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

% Solve shit
x_diff = 56.40783*1e-3;
y_diff = 371.4759*1e-3;

% Torque Offsets
% T_s = [0.2;0.15;0.14;0.12]; % Starting torque for motors (from non-cabling testing)
% T_s = [0.2;0.2;0.2;0.2]; % Starting torque for motors
% T_s = [0.3;0.3;0.2;0.2]; % Starting torque for motors
% T_s = 0.05*ones(4,1);%[0.1;0.1;0.1;0.1]; % Starting torque for motors
precV = 1; % Precision for evaluating velocity
precT = 1; % Precision for evaluating torque
precV = 2;
precE = 3;
precF = 0;
f_static    = [0.0840;0.0820;0.1040;0.0780]*(1/R);
f_loss      = [2.195;2.295;2.245;1.845];


%% Initialize variables
x   = 0.2;                        % Desired x-position
y   = 0;                          % Desired y-position
phi = deg2rad(0);                            % Desired phi-angle [radians]

q_d         = [x;y;phi];            % Desired pose
q           = [0;0;0];

l0 = [1.2260 1.1833 1.1833 1.2260]';

errorEncountered = false;           % Initialize error counter bit

%% Control

% Kp = zeros(3,1);
% Ki = zeros(3,1);
% Kd = zeros(3,1);
% 
% Tk = [0.5255;0.3458;0];
% Kk = [500;450;10];
% for i= 1:3
%     Kp(i) = Kk(i)*0.3;
%     Ki(i) = Kp(i)/(0.85*Tk(i));
%     Kd(i) = Kp(i)*Tk(i)*0.12;
% end
% 
% Kp = diag(Kp);
% Ki = diag(Ki);
% Ki(3,3)=0;  % no I on angle
% Kd = diag(Kd);

e_int   = zeros(3,1);
e_tol   = [0.5;0.5;0.5];



% PI REG PARAM
% Kp = 0.6*diag([500*0.45 50 5]);
% Ki = 0.5*diag([Kp(1)/(0.85*Tk(1)) 0 0]);

% Kp = diag([100 100 10]);
% Ki = diag([0 0 0]);
% Ki = 0.5*diag([Kp(1)/(0.85*Tk(1)) 0 0]);
% Kd = diag([0 0 0]);

% TODO  : FINN KRITISK FORSTERKNING FOR Y
%       : TEST PI PÅ Y
%       : TUNE PHI GAINS
%       : evt: DERIVATVIRKNING

%% ELIAS SIN PID

Kp = diag([200; 200; 5]);
Ki = diag([0;0;0]);
Kd = diag([0 0 0]);

%% Preallocation of vectors
f_prev      = f_ref*ones(4,1);  % Memory Allocation for prev cable forces

l           = zeros(4,1);       % Memory Allocation for cable lengths
l_fk        = zeros(4,1);       % Memory Allocation for cable lengths
l_enc       = zeros(4,1);
w_d         = zeros(4,1);
l_dot       = zeros(4,1);       % Memory Allocation for cable velocities
theta_m_dot = zeros(4,1);
pos_rad     = zeros(4,1);
vel         = zeros(4,1);

%% Logging 
iter = 100;
total_time  = zeros(1,iter);
count       = 1;
timeVec     = zeros(1,iter);
timeSca     = 0;


q_log       = zeros(3,iter);
qd_log      = zeros(3,iter);
e_log       = zeros(3,iter);    
e_int_log   = zeros(3,iter);   

%% Sine testing - ellipse
omega = 3;
h_test = 0.2;
Ly = 0.1;
Lx = 0.1;
time_test = 0;
h_sample = 0.06;


ref_time = 0.07;

fieldNames = fieldnames(ODriveStruct);
%% Control Loop
s_start = tic;
while errorEncountered == false && count <= iter
t_loop = tic;
    %% Estimate Cable Lengths
    % Get position and velocity of motors
    [pos, vel] = TEST_getEncoderReading(ODriveStruct);
    
    for k = 1:4      
        pos_rad(k) = pos(k)*2*pi;
        l_enc(k) = pos_rad(k)*R*motorsigns(k);
        l(k) = l0(k) + l_enc(k);
        w_d(k) = pos(k)*P;
        l_fk(k) = l(k) - sqrt(sqrt(w_d(k)^2+y_diff^2)^2 + x_diff^2);

        % Cable Velocity
        theta_m_dot(k)  = vel(k)*2*pi*motorsigns(k);
        l_dot(k)        = theta_m_dot(k)*R;
    end


    %% Controller
    % Calculate current and desired pose of the platform
    q_0 = init_fk_estimate(a,b,l_fk);
    q = p_forward_kinematics(a,b,l_fk,q_0,r_p);


    % % Test sine
    bla = toc(s_start);
    % q_d         = [Lx*sin(omega*bla);Ly*cos(omega*bla)-0.05;phi];            % Desired pose
    % q_d         = [0;Ly*cos(omega*bla);phi];
    % q_d         = [Lx*sin(omega*bla);0;phi];

    % rad2deg(q(3));

    % Calculate Structure Matrix
    [~,betar]   = p_inverse_kinematics(a,b,q, r_p);
    A_t         = structure_matrix(a,b,q,r_p,betar);
    A_t_pseudo  = pinv(A_t); 
    A_pseudo    = pinv(A_t');

    % 
    q_dot = -A_pseudo*l_dot;

    % Calculate Errors
    e           = q_d - q;
    e_dot       = q_dot;

     % Anti Theft System
    if sign(e_int(1))*e_int(1) > e_tol(1)
        e_int(1) = e_tol(1)*sign(e_int(1));
    end

    if sign(e_int(2))*e_int(2) > e_tol(2)
        e_int(2) = e_tol(2)*sign(e_int(2));
    end

    if sign(e_int(3))*e_int(3) > e_tol(3)
        e_int(3) = e_tol(3)*sign(e_int(3));
    end

    % Desired wrench (TESTE KONTROLLER)
    w_cP = Kp*e;
    w_cI = Ki*e_int;
    w_cD = Kd*e_dot;
    w_c =  w_cP + w_cI + w_cD
    % w_c = PID_Fossen(e, e_dot,e_int)
    % tic

    % Force Allocation
    [f,~,flag] = ForceAllocIterativeSlack(A_t',f_min,f_max,f_ref,f_prev,w_c);

    
    f_s = ...
    (~(ones(4,1)&fix(vel*10^precV)/10^precV)...
    &any(fix(e*10^precE)/10^precE)).*f_static;
    % tmp = ones(4,1)&fix(vel*10^precV)/10^precV
    f_f = f_s + f_loss
    
    % f0 = sign(A_pseudo*w_c)*f_f;
    lol = A_t_pseudo*w_c;
    f0 = sign(fix((lol)*10^precF)/10^precF).*f_f;
 

    T = (f+f0)*R.*motorsigns*(-1); % fordi motorsigns er for endring i kabelendring og ikke rotasjonsretning på torque
    % tic
    % offset = ~(ones(4,1)&(fix(vel*10^precV)/10^precV|~(fix(T*10^precT)/10^precT))).*sign(T).*T_s;
    % t_sol = toc
    % T = T + offset;


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

    % time_test = time_test + h_test;

   

    % Update Integral Error
    e_int = e_int + e*toc(t_loop);

    
    
    % Logging
    % total_time(count)   = toc(t_loop);
    q_log(:,count)      = q;
    qd_log(:,count)     = q_d;
    e_log(:,count)      = e; 
    e_int_log(:,count)  = e_int; 
    

    % Fixed sampling time
    % time_diff = ref_time - toc(t_loop);
    % if sign(time_diff) >= 0
    %     pause(time_diff)
    % end
    total_time(count)   = toc(t_loop);
    timeSca = timeSca + total_time(count);
    timeVec(count) = timeSca;

    count               = count + 1;
end
%% Save logged data
save("PlotData/PoseData.mat", "q_log","qd_log","e_log","e_int_log", "timeVec");
% Plot of total time and Hertz
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
T = [0.2;-0.2;0.2;-0.2];
    for k = 1:length(fieldNames)
        fieldName = fieldNames{k}; % Current field name as a string
        currentSerialPort = ODriveStruct.(fieldName); % Access the current serial port using dynamic field names
        setMotorTorque(T(k), currentSerialPort);
    end


end