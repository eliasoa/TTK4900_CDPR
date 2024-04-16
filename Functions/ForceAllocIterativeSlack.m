function [f,w_resultant] = ForceAllocIterativeSlack(A,f_min, f_max, f_ref, f_prev, w_ref)

% Function for calculating optimal force distributions of a Cable Driven Parallel Robot.
%
% Inspired by Einar Ueland, Thomas Sauder and Roger Skjetne, Department of
% Marine Technology; Norwegian University of Science and Technology; 
% Centre for Autonomous Marine Operations and Systems (NTNU AMOS);
% NO-7491 Trondheim, Norway
% SINTEF Ocean; NO-7465 Trondheim, Norway  
%
% Created by Magnus Grøterud, 13.04.2024

%% TESTING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clear all
% clc
% 
% init_CDPR_Params
% SymbolicTemplateForceAlloc
% 
% q0 = [-0.4;-0.2;0];
% a = CDPR_Params.SGM.FrameAP;
% b = CDPR_Params.SGM.BodyAP.RECTANGLE;
% 
% A = WrenchMatrix_V2(a,b,q0); 
% 
% f_min = 5;
% f_max = 60;
% f_ref = 25;
% 
% fMinVec = f_min*ones(4,1);
% fMaxVec = f_max*ones(4,1);
% 
% 
% w_ref = [-5;5;0];
% f_prev = f_ref*ones(4,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Force Allocation Algorithm

m = 3;          % Number of Controllable DOFs
n = 4;          % Number of actuating cables
p = 2;          % P-norm value

% Defining the Optimization Matrices
W = A';         % Optimization Matrix
% H = pinv(W);    % Pseudo inverse of W
Q = eye(m);     % Weighting matrix for slack variable
A = [W Q];      % Optimization matrix with slack

% Params
c       = 0.1;              % Parameter adjusting how fast the cost function for the standard formulation increases
epsilon = 10^(-3);          % Parameter adjusting the curvature of the cost function for the slacked formulation
b       = 200;              % Parameter steering the gradient of the cost term for the slacked formulation
c_phi   = 1;%10^(-3);          % Parameter for checking merit function value


%% Newtons Method on the KKT Conditions
% Initialization
iter    = 0;                    % Initializing iteration counter
iterMax = 1000;                 % Maximum Iterations

f       = f_prev;               % Initial Force 
f0      = f_ref;                % Defining the reference force
s       = zeros(m,1);           % Initial Slack Variables
x       = [f;s];                % Optimization Variables

lambda  = zeros(m,1);           % Initial Lagrangian Multipliers
lambda_prev = zeros(m,1);       % 
% alpha = (f_max - f_min)/2;      % Normalization Factor to avoid numerical accuracy issues

z       = [x;lambda];           % Initial Full state 
tol     = 5e-5;                 % Merit Function Tolerance

tic
while iter <= iterMax
    % 1) Calculate Newton Step  
       
    GradientX   = GradientObj(x, f_min, f_max, f0, p, c,b,epsilon)';    % Calculate gradient of objective function 
    HessianX    = HessianObj(x, f_min, f_max, f0, p, c,b,epsilon);      % Calculate hessian of objective function

    zeroBlock = zeros(m,m);
    A_KKT = [HessianX A';A zeroBlock];                                  % KKT Matrix
    B_KKT = [GradientX;A*x - w_ref];                                    % 

    d_k = A_KKT\-B_KKT;                                                 % Calculate Newton Step
    d_k(length(x)+1:length(x)+m) = d_k(length(x)+1:length(x)+m)- lambda_prev;

    % 2) Calculate Step Length
    kappa           = 1;                    % Initial Step Size for Newton Step
    % disp("Merit Function Value Iteration " + string(iter) + ":")

    % Linesearch with Merit Function
    phiMerit        = MeritFunction(z, GradientX, A, w_ref);
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

    while phi_kappa > phiMerit + c_phi*kappa*D_phi && iterMerit <= iterMeritMax
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

f = z(1:4);
s = z(5:7);
w_resultant = w_ref - Q*s;


%% Functions

% Function for calculating the merit function
function phi_merit = MeritFunction(z,grad_g, A, w_ref)
    x_local         = z(1:7);
    lambda_local    = z(8:10);
    phiVec          = [grad_g + A'*lambda_local;A*x_local - w_ref];
    phi_merit       = norm(phiVec, Inf);
end

% Function for calculating the "Directional Derivative" of merit function
% (Markus Grasmair, Department of Mathematics, 
% Norwegian University of Science and Technology,
% Trondheim, Norway)
%
% (OBS: Tror ikke denne er helt riktig)
function D_phi_merit = D_MeritFunc(z,grad_g, A, w_ref, d_k)
    x_local         = z(1:7);
    lambda_local    = z(8:10);
    p_k             = d_k(1:7);  
    DphiVec         = [grad_g'*p_k + A'*lambda_local;A*x_local - w_ref]; % VEEELDIG USIKKER PÅ DENNE
    D_phi_merit     = norm(DphiVec, Inf);
end

end



