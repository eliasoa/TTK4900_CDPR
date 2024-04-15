% function f = ForceAllocIterative(A,a,b,f_min, f_max, f_ref, w_ref)

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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Force Allocation Algorithm

m = 3;          % Number of Controllable DOFs
n = 4;          % Number of actuating cables
p = 2;          % P-norm value

% Defining the Optimization Matrices
W = A'; 
H = pinv(W);

f0 = f_ref;

% Params
% sigma   = norm(H,1);
% lambda  = max(abs(f_max - f0), abs(f0 - f_min));
c1      = 0.1;                % Constant enabling adjustments in how fast the cost function increases
c2      = c1;               % Constant enabling adjustments in how fast the cost function increases
epsilon = 10^(-3);          % For slack version, not implemented yet.

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
f       = f0*ones(n,1);         % Initial Force
lambda  = zeros(m,1);
lambda_prev = zeros(m,1);
alpha = (f_max - f_min)/2;      % Normalization Factor to avoid numerical accuracy issues

z       = [f;lambda];           % z = [f;lambda]
tol     = 5e-5;                 % Merit Function Tolerance
tic
while iter <= iterMax
    % 1) Calculate Newton Step
    f = z(1:n);
    grad_g_f = GradientObjFunc(f, f_min, f_max,f0, c1,c2,p,alpha);
    H_g_f = HessianObjFunc(f, f_min, f_max,f0, c1,c2,p,alpha);

    % [ObjFun, GradObj, HessObj] = calculate_g_f(f(1), f(2), f(3), f(4), f_min, f_max, f0, c1, c2);
    
    A = [H_g_f W';W zeros(3,3)];
    B = [grad_g_f;W*f - w_ref];

    d_k = A\-B;
    d_k(n+1:n+m) = d_k(n+1:n+m)- lambda_prev;

    % 2) Linesearch with Merit Function
    kappa           = 1;                    % Initial Step Size for Newton Step
    disp("Merit Function Value Iteration " + string(iter) + ":\n")
    phiMerit        = MeritFunction(z, grad_g_f, W, w_ref)
    phi_kappa       = MeritFunction(z+kappa*d_k, grad_g_f, W, w_ref); 
    D_phi           = D_MeritFunc(z,grad_g_f, W, w_ref, d_k);

    iterMerit       = 0;
    iterMeritMax    = 100;

    while phi_kappa > phiMerit + D_phi && iterMerit <= iterMeritMax
        kappa = kappa-0.01;
        if kappa <= 0
            kappa = 0;
            break
        end
        
        phi_kappa       = MeritFunction(z+kappa*d_k, grad_g_f, W, w_ref); 
        iterMerit = iterMerit + 1;

        if iterMerit > iterMax
            disp("Too many iterations (Merit Function)")
        end
    end
    
    
    % 3) Perform Newton step
    z = z + kappa*d_k; 
    lambda_prev = z(n+1:n+m);
    
    % 4) Check if  merit function is below the predetermined tolerance threshold
    if phiMerit < tol 
        disp("Algorithm Terminated after " + string(iter) + " iterations.")
        break
    end
    
    % 5) Update main iterations
    iter = iter + 1;
    if iter > iterMax
        disp("Too many iterations (Newton Step)")
    end
end
toc

f = z(1:4)


%% FUNCTIONS
% Function for calculating the objective function
function g_f = ObjectiveFunction(f, f_min, f_max,f_0, c1,c2,p,alpha)
g_f = 0;                   % Memory Allocation


for i=1:4
    objFunc = (abs(f(i) - f_0)^p)/alpha - c1*log(f(i) - f_min) - c2*log(f_max - f(i));
    g_f = g_f + objFunc;
end
end


% Function for calculating the Gradient of the objective function
function grad_g_f = GradientObjFunc(f, f_min, f_max,f_0, c1,c2,p,alpha)
grad_g_f = zeros(4,1);      % Memory Allocation

for i= 1:4
    grad_g_f(i) = (p*abs(f(i) - f_0))/alpha^p - c1/(f(i) - f_min) - c2/(f_max - f(i));  
end
end

% Function for calculating the Hessian of the objective function
function H_g_f = HessianObjFunc(f, f_min, f_max,f_0, c1,c2,p,alpha)
H_g_f = zeros(4,1);          % Memory Allocation

% Calculating the diagonal of the Hessian
for i=1:4
    % Handling division by zero
    if f(i) == f_0
        H_g_f(i) =  c1/(f(i) - f_min)^2 - c2/(f_max - f(i));
    else
        H_g_f(i)   =  (p*(f(i) - f_0))/(alpha*abs(f(i) - f_0)) + c1/(f(i) - f_min)^2 - c2/(f_max - f(i)); 
    end
end

H_g_f = diag(H_g_f);
end

% Function for calculating the merit function
function phi_merit = MeritFunction(z,grad_g, W, w_ref)
    f       = z(1:4);
    lambda  = z(5:7);
    phiVec = [grad_g + W'*lambda;W*f - w_ref];
    phi_merit = norm(phiVec, Inf);
end

% Function for calculating the "Directional Derivative" of merit function
% (Markus Grasmair, Department of Mathematics, 
% Norwegian University of Science and Technology,
% Trondheim, Norway)
%
% (OBS: Tror ikke denne er helt riktig)

function D_phi_merit = D_MeritFunc(z,grad_g, W, w_ref, d_k)
    f               = z(1:4);
    lambda          = z(5:7);
    p_k             = d_k(1:4);
    DphiVec         = [grad_g'*p_k + W'*lambda;W*f - w_ref]; % VEEELDIG USIKKER PÅ DENNE
    D_phi_merit     = norm(DphiVec, Inf);
  
end



% end



