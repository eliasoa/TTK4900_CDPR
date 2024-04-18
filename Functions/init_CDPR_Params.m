%% Initializing General Robot Parameters

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               PARAMETERS THAT CAN BE CHANGED               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial platform position
x0       = -0.2;
y0       = -0.2;
theta0   = 0.0;
xd0      = 0.0;
yd0      = 0.0;
thetad0  = 0.0;

r0 = [x0; y0];
q0 = [r0; theta0];
qd0 = [xd0;yd0; thetad0];

h = 0.01;       % Sampling time (can be overwritten in main_script)

% Spool Parameters
Rs  = 0.02;                 % Spool radius
P   = 2.9*10^(-3);          % Pitch of spool
% d   =                       % Horizontal distance from cable outlet of spool to pulley  
% h0  =                       % Vertical Height from spool to pulley
% hs  =  sqrt(d^2 + h0^2);    % Length of cable between spool and pulley (at x=0, home position) 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Physical parameters 
g               = 9.81;                     % m/s^2

% Mobile Platform parameters

% Lengths of mobile platform
MP_len_x        = 0.15;                   % [m]
MP_len_y        = 0.05;                      % [m]
        
% Dimension of the frame
F_len_x         = 1.46;                       % [m]                
F_len_y         = 1;                         % [m]

d               = 0.01;                     % m
V               = MP_len_x*MP_len_y*d;                    % m^3
rho             = 2710;                     % kg/m^3
mp               = V*rho;                   % kg
mp               = 0.07;                    % Målt med vekta til Solve
Izz              = 1/12*mp*(MP_len_x^2+MP_len_y^2);       % kg m^2
dtx = 0.0;                                  % Translational dampening coefficient in the x-direction
dty = 0.0;                                  % Translational dampening coefficient in the y-direction
dr  = 0.0;                                  % Rotational dampening coefficient about the z-axis

% Wrench due to gravity
Wp              = mp*[0 -g 0]';

%% Cable attachment points PULLEY (In INERTIA coordinates) CONSTANT
%% SØLVE
h_pb = 30.2*1e-3; % [m] - height of pillow block bearing
a_pb = 127*1e-3;  % [m] - length of pillow block bearing

x = 0;
y = 0;
r = [x; y];
theta = 0;
q = [r;theta]; % Generalized coordinates of platform

profile_side_length = 30*1e-3; % [m]
fspace_w = 1400*1e-3; % [m]
fspace_h = 1000*1e-3; % [m]

% length_drum_x = fspace_w/2+profile_side_length+a_pb/2-1*1e-3;
length_drum_x = [sqrt(1.52^2 + 2.99^2);
                 sqrt(2.61^2 + 1.83^2);
                 sqrt(0.98^2 + 0.8^2);
                 sqrt(0.65^2 + 0.55^2)]*10^(-3);
length_drum_y = profile_side_length/2 + h_pb;


dc1 = [-length_drum_x(1); -length_drum_y];
dc2 = [-length_drum_x(2);  length_drum_y];
dc3 = [ length_drum_x(3);  length_drum_y];
dc4 = [ length_drum_x(4); -length_drum_y];

beta_e = 0.17896;
r_d = 20*1e-3;
% beta_e*180/pi

de1 = dc1 + [r_d*cos(pi+beta_e);   r_d*sin(pi+beta_e)];
de2 = dc2 + [r_d*cos(pi-beta_e);   r_d*sin(pi-beta_e)];
de3 = dc3 + [r_d*cos(beta_e);      r_d*sin(beta_e)];
de4 = dc4 + [r_d*cos(2*pi-beta_e); r_d*sin(2*pi-beta_e)];

r_p = 12*1e-3;
angle_profile_side_length = 40*1e-3;
pulley_mounting_hole_offset = 4*1e-3;
pulley_mh_to_wh = [26.4; 41.9]*1e-3; % Length between the pulley assembly 
                                % mounting hole to the pulley wheel hole
length_pulley_x = fspace_w/2+profile_side_length + ...
                  angle_profile_side_length - ...
                  pulley_mounting_hole_offset - ...
                  pulley_mh_to_wh(1);
length_pulley_y = fspace_h/2-angle_profile_side_length-pulley_mh_to_wh(2);

pc1 = [-length_pulley_x; -length_pulley_y];
pc2 = [-length_pulley_x;  length_pulley_y-30*1e-3];
pc3 = [ length_pulley_x;  length_pulley_y-30*1e-3];
pc4 = [ length_pulley_x; -length_pulley_y];
pc = [pc1 pc2 pc3 pc4];

pe1 = pc1 + [r_p*cos(pi+beta_e);   r_p*sin(pi+beta_e)];
pe2 = pc2 + [r_p*cos(pi-beta_e);   r_p*sin(pi-beta_e)];
pe3 = pc3 + [r_p*cos(beta_e);      r_p*sin(beta_e)];
pe4 = pc4 + [r_p*cos(2*pi-beta_e); r_p*sin(2*pi-beta_e)];

a1 = pc1 - [r_p; 0];
a2 = pc2 - [r_p; 0];
a3 = pc3 + [r_p; 0];
a4 = pc4 + [r_p; 0];

% a = [a1 a2 a3 a4];
 a = [-0.7560   -0.7560    0.7560    0.7560;
   -0.4181    0.4181    0.4181   -0.4181];

% a = [ -0.7516   -0.7516    0.7516    0.7516;
%       -0.4181    0.4181    0.4181   -0.4181];

% a               = [a1 a2 a3 a4];

% a               = [0.2 -0.2; 0 0]; % 1 DoF test rigg
% b               = [0 0; 0 0]; 

% Cable attachment point PLATFORM

% % RECTANGLE FOR TESTRIGG4
% b1              = [-30+4.12;-10.24] *1e-3;  
% b2              = [-30+4.36;11.24]  *1e-3;   
% b3              = [30-4.52;11.24]   *1e-3;    
% b4              = [30-4.54;-10.24]  *1e-3;
% b_rectangle     = [b1 b2 b3 b4];





% RECTANGLE
b1              = [-MP_len_x/2;-MP_len_y/2];  
b2              = [-MP_len_x/2;MP_len_y/2];   
b3              = [MP_len_x/2;MP_len_y/2];    
b4              = [MP_len_x/2;-MP_len_y/2];
b_rectangle               = [b1 b2 b3 b4];

% TRIANGLE
b1              = [0;-MP_len_y/2];
b2              = [-MP_len_x/2;MP_len_y/2];   
b3              = [MP_len_x/2;MP_len_y/2];     
b4              = b1;
b_triangle               = [b1 b2 b3 b4];

% TRAPEZOIDAL
b_trapez        = [ -0.0250   -0.0750    0.0750    0.0250;
                    -0.0100    0.0100    0.0100   -0.0100];


%% Discrete Linear Model
A_c = [0  0 0  1       0       0;        % 
       0  0 0  0       1       0;
       0  0 0  0       0       1;
       0  0 0 -dtx/mp  0       0;
       0  0 0  0      -dty/mp  0;
       0  0 0  0       0      -dr/Izz];

B_c = [0    0    0;
       0    0    0;
       0    0    0;
       1/mp 0    0;
       0    1/mp 0;
       0    0    1/Izz;];
d_c = [ 0;
        0;
        0;
        0;
       -g;
        0];

% Augmented Model for Integral States
A_c_aug = [A_c    zeros(6,6);
           -eye(6) zeros(6,6)];
B_c_aug = [B_c; zeros(6,3)];
d_c_aug = [d_c; zeros(6,1)];
zeroI = [zeros(6,6);eye(6)];

K_d = B_c\eye(6);
K_f = [180*diag([1,1,1]) 10*diag([1,1,1])];
K_r = pinv((B_c*K_f-A_c)\B_c);
K_a = [diag([-10,-10,-1]) zeros(3,3)];

% Motorsign (CHANGE IF NEEDED)
motorsign0 = -1;
motorsign1 = 1;
motorsign2 = -1;
motorsign3 = 1;

motorsigns = [motorsign0;motorsign1;motorsign2;motorsign3];



%% Structs

% Standard Geometric Model Params
CDPR_BodyAnchorPoints = struct("RECTANGLE", b_rectangle, ...
                               "TRIANGLE", b_triangle, ...
                               "TRAPEZOID", b_trapez, ...
                               "TEST", b_rectangle);
CDPR_SGM = struct("FrameAP", a, ...
                  "BodyAP", CDPR_BodyAnchorPoints);

% Spool Params
CDPR_SpoolParams  = struct("SPOOL_RADIUS", Rs, ...
                           "PITCH", P);%, ...
                           %"SPOOL_PULLEY_LENGTH",hs);

% Control Params
CDPR_ControlParams = struct("K_d", K_d, ...
                            "K_f", K_f, ...
                            "K_r", K_r, ...
                            "K_a", K_a, ...
                            "d_c", d_c);

% System Matrices
CDPR_SystemMatrices = struct("A_c_aug", A_c_aug, ...
                             "B_c_aug", B_c_aug, ...
                             "d_c_aug", d_c_aug, ...
                             "zeroI", zeroI);

% General Parameters
CDPR_GenParams = struct("SAMPLING_TIME", h, ...
                        "MASS_PLATFORM", mp, ...
                        "MOTOR_SIGNS", motorsigns, ...
                        "SpoolParams", CDPR_SpoolParams);
% More?



% Combine all structs into a master struct
CDPR_Params = struct("SGM", CDPR_SGM, ...
                     "ControlParams", CDPR_ControlParams, ...
                     "SystemMatrices", CDPR_SystemMatrices, ...
                     "Gen_Params", CDPR_GenParams);

