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
Rs = 0.02;      % Spool radius


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Physical parameters 
g               = 9.81;                     % m/s^2

% Mobile Platform parameters

% Lengths of mobile platform
MP_len_x        = 0.15;                      % [m]
MP_len_y        = 0.02;                      % [m]
        
% Dimension of the frame
F_len_x         = 1.4;                       % [m]                
F_len_y         = 1;                         % [m]

d               = 0.01;                     % m
V               = MP_len_x*MP_len_y*d;                    % m^3
rho             = 2710;                     % kg/m^3
mp               = V*rho;                   % kg
mp               = 0.1;                   % TEMPORARY
Izz              = 1/12*mp*(MP_len_x^2+MP_len_y^2);       % kg m^2
dtx = 0.0;                                  % Translational dampening coefficient in the x-direction
dty = 0.0;                                  % Translational dampening coefficient in the y-direction
dr  = 0.0;                                  % Rotational dampening coefficient about the z-axis

% Wrench due to gravity
Wp              = mp*[0 -g 0]';

% Cable attachment points PULLEY (In INERTIA coordinates) CONSTANT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   OBS: LAG DENNE MER DYNAMISK/MINDRE HARDKODA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% a1              = [-F_len_x/2;-F_len_y/2];  
% a2              = [-F_len_x/2;F_len_y/2];
% a3              = [F_len_x/2;F_len_y/2];    
% a4              = [F_len_x/2;-F_len_y/2];  
% a               = [a1 a2 a3 a4];

a               = [0.2 -0.2; 0 0]; % 1 DoF test rigg
b               = [0 0; 0 0]; 

% Cable attachment point PLATFORM

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
b1              = [-MP_len_x/4;-MP_len_y/2];
b2              = [-MP_len_x/2;MP_len_y/2];   
b3              = [MP_len_x/2;MP_len_y/2];     
b4              = [MP_len_x/4;-MP_len_y/2];   
b_trapez      = [b1 b2 b3 b4];


% Discrete Linear Model
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
motorsign1 = 1;
motorsign2 = 1;
motorsign3 = -1;
motorsign4 = -1;



%% Structs

% Standard Geometric Model Params
CDPR_BodyAnchorPoints = struct("RECTANGLE", b_rectangle, ...
                               "TRIANGLE", b_triangle, ...
                               "TRAPEZOID", b_trapez, ...
                               "TEST", b);
CDPR_SGM = struct("FrameAP", a, ...
                  "BodyAP", CDPR_BodyAnchorPoints);

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
                        "SPOOL_RADIUS", Rs, ...
                        "Platform_mass", mp);
% More?



% Combine all structs into a master struct
CDPR_Params = struct("SGM", CDPR_SGM, ...
                     "ControlParams", CDPR_ControlParams, ...
                     "SystemMatrices", CDPR_SystemMatrices, ...
                     "Gen_Params", CDPR_GenParams);

