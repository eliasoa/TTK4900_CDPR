% function f = ForceAllocIterativeSlack(A,a,b,f_min, f_max, f_ref, f_prev, w_ref)

% Function for calculating optimal force distributions of a Cable Robot.
% Inspired by Einar Ueland, Thomas Sauder and Roger Skjetne, Department of
% Marine Technology; Norwegian University of Science and Technology; 
% Centre for Autonomous Marine Operations and Systems (NTNU AMOS);
% NO-7491 Trondheim, Norway
% SINTEF Ocean; NO-7465 Trondheim, Norway  
%
% Created by Magnus Grøterud, --.04.2024

%% TESTING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
clc

init_CDPR_Params
SymbolicTemplateForceAlloc

q0 = [0.4;0;0];
a = CDPR_Params.SGM.FrameAP;
b = CDPR_Params.SGM.BodyAP.RECTANGLE;

A = WrenchMatrix_V2(a,b,q0); 

f_min = 5;
f_max = 60;
f_ref = 25;

fMinVec = f_min*ones(4,1);
fMaxVec = f_max*ones(4,1);


w_ref = [-10;0;0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Force Allocation Algorithm

m = 3;          % Number of Controllable DOFs
n = 4;          % Number of actuating cables
p = 2;          % P-norm value

% Defining the Optimization Matrices
W = A';         % Optimization Matrix
H = pinv(W);    % Pseudo inverse of W
Q = eye(m);     % Weighting matrix for slack variable

A = [W Q];      % Optimization matrix with slack

% Defining the reference force
f0 = f_ref;

% Params
% sigma   = norm(H,1);
% lambda  = max(abs(f_max - f0), abs(f0 - f_min));
c       = 0.1;              % Constant enabling adjustments in how fast the cost function increases
epsilon = 10^(-3);          % For slack version, not implemented yet.
b       = 200;              % 

% delta 

%% Defining the constraints (SYMBOLIC)
% syms f1 f2 f3 f4
% f_syms = [f1;f2;f3;f4];
% 
% h_c = [f_syms - fMinVec;f_max-f_syms];        % Cable Force Constraint
% grad_h = jacobian(h_c,f_syms);            % Derivative of h_c

%% Defining the objective function

% % TEST
% f = [20;28;23;18];
% 
% g_f = ObjectiveFunction(f, f_min, f_max, f0, c1, c2, p)
% grad_g_f = GradientObjFunc(f, f_min, f_max,f0, c1,c2,p)
% H_g_f = HessianObjFunc(f, f_min, f_max,f0, c1,c2,p)
% 
% z = zeros(n+m,1);
% PHI = MeritFunction(grad_g_f, W, z, w_ref)

%% Newtons Method on the KKT Conditions
% Initialization
iter    = 0;
iterMax = 1000;

f       = f0*ones(n,1);         % Initial Force (SHOULD BE F_PREV WHEN RUNNING)
s       = zeros(m,1);           % Initial Slack Variables
x       = [f;s];                % Optimization Variables

lambda  = zeros(m,1);
lambda_prev = zeros(m,1);
% alpha = (f_max - f_min)/2;      % Normalization Factor to avoid numerical accuracy issues

z       = [x;lambda];           % z = [f;lambda]
tol     = 5e-5;                 % Merit Function Tolerance

tic
while iter <= iterMax
    % 1) Calculate Newton Step  
      
    GradientX   = GradientObj(x, f_min, f_max, f0, p, c,b,epsilon)';
    HessianX    = HessianObj(x, f_min, f_max, f0, p, c,b,epsilon); 

    zeroBlock = zeros(m,m);
    A_KKT = [HessianX A';A zeroBlock];
    B_KKT = [GradientX;A*x - w_ref];

    d_k = A_KKT\-B_KKT;
    d_k(length(x)+1:length(x)+m) = d_k(length(x)+1:length(x)+m)- lambda_prev;

    % 2) Linesearch with Merit Function
    kappa           = 1;                    % Initial Step Size for Newton Step
    disp("Merit Function Value Iteration " + string(iter) + ":")
    phiMerit        = MeritFunction(z, GradientX, A, w_ref)
    phi_kappa       = MeritFunction(z+kappa*d_k, GradientX, A, w_ref); 
    D_phi           = D_MeritFunc(z,GradientX, A, w_ref, d_k);

    % Check if  merit function is below the predetermined tolerance threshold
    if phiMerit < tol 
        disp("Algorithm Terminated after " + string(iter) + " iterations.")
        break
    end
    
    % Else, update 
    iterMerit       = 0;
    iterMeritMax    = 100;

    while phi_kappa > phiMerit + D_phi && iterMerit <= iterMeritMax
        kappa = kappa-0.01;
        if kappa <= 0
            kappa = 0;
            break
        end

        phi_kappa       = MeritFunction(z+kappa*d_k, GradientX, A, w_ref); 
        
        % Update Iteration For Merit Function
        iterMerit = iterMerit + 1;
        if iterMerit > iterMeritMax
            disp("Too many iterations (Merit Function)")
        end
    end
    
    % 3) Perform Newton step
    z = z + kappa*d_k; 

    % Update Previous Lambda
    lambda_prev = z(length(x)+1:length(x)+m);

    % Extract State
    x = z(1:length(x));
    
    % 5) Update main iterations
    iter = iter + 1;
    if iter > iterMax
        disp("Too many iterations (Newton Step)")
    end
end
toc

f = z(1:4)


%% Functions

% Function for calculating the merit function
function phi_merit = MeritFunction(z,grad_g, A, w_ref)
    x       = z(1:7);
    lambda  = z(8:10);
    phiVec = [grad_g + A'*lambda;A*x - w_ref];
    phi_merit = norm(phiVec, Inf);
end

% Function for calculating the "Directional Derivative" of merit function
% (Markus Grasmair, Department of Mathematics, 
% Norwegian University of Science and Technology,
% Trondheim, Norway)
%
% (OBS: Tror ikke denne er helt riktig)
function D_phi_merit = D_MeritFunc(z,grad_g, A, w_ref, d_k)
    x               = z(1:7);
    lambda          = z(8:10);
    p_k             = d_k(1:7);
    DphiVec         = [grad_g'*p_k + A'*lambda;A*x - w_ref]; % VEEELDIG USIKKER PÅ DENNE
    D_phi_merit     = norm(DphiVec, Inf);
end





