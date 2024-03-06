%% Initializing General Robot Parameters

% Physical parameters 
g               = 9.81;                     % m/s^2

% Mobile Platform parameters

% Lengths of mobile platform
MP_len_x        = 0.15;                      % [m]
MP_len_y        = 0.02;                      % [m]
        
% Dimension of the frame
F_len_x         = 1.4;                       % [m]                
F_len_y         = 1;                         % [m]
l               = MP_len_x;                 % m
h               = MP_len_y;                 % m
d               = 0.01;                     % m
V               = l*h*d;                    % m^3
rho             = 2710;                     % kg/m^3
mp               = V*rho;                   % kg
mp               = 0.250;                   % TEMPORARY
Izz              = 1/12*mp*(l^2+h^2);       % kg m^2
dtx = 0.0;                                  % Translational dampening coefficient in the x-direction
dty = 0.0;                                  % Translational dampening coefficient in the y-direction
dr  = 0.0;                                  % Rotational dampening coefficient about the z-axis

% Wrench due to gravity
Wp              = mp*[0 -g 0]';



% Cable attachment points PULLEY (In INERTIA coordinates) CONSTANT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   OBS: LAG DENNE MER DYNAMISK/MINDRE HARDKODA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
a1              = [-F_len_x/2;-F_len_y/2];  
a2              = [-F_len_x/2;F_len_y/2];
a3              = [F_len_x/2;F_len_y/2];    
a4              = [F_len_x/2;-F_len_y/2];  
a               = [a1 a2 a3 a4];

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

K_d = B_c\eye(6);
K_f = [180*diag([1,1,1]) 10*diag([1,1,1])];
K_r = pinv((B_c*K_f-A_c)\B_c);
K_a = [diag([-10,-10,-1]) zeros(3,3)];



%% Structs

% Standard Geometric Model Params
CDPR_BodyAnchorPoints = struct("RECTANGLE", b_rectangle, ...
                               "TRIANGLE", b_triangle, ...
                               "TRAPEZOID", b_trapez);
CDPR_SGM = struct("FrameAP", a, ...
                  "BodyAP", CDPR_BodyAnchorPoints);

% Control Params
CDPR_ControlParams = struct("K_d", K_d, ...
                            "K_f", K_f, ...
                            "K_r", K_r, ...
                            "K_a", K_a, ...
                            "d_c", d_c);

% More?



% Combine all structs into a master struct
CDPR_Params = struct("SGM", CDPR_SGM, ...
                     "ControlParams", CDPR_ControlParams);

