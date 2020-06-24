%% General

% Simulation
Nruns = 1;
Tsim = 100; % [s]
pars.Tsim = Tsim;

x0 = [-10; 10; pi/2; 0; 0];

%% System

x_min = -10; % [m]
x_max = 10; % [m]
y_min = -10; % [m]
y_max = 10; % [m]

% Mass and moment of inertia
pars.m = 10; % [kg]
pars.I = 1; % [kg m²]

pars.dimInput = 2;
pars.dimOutput = 5;
pars.dimState = 5;

l = pars.dimInput;
p = pars.dimOutput;
 
%% Controller: common

% Sampling time
delta_t = 0.01; % [s]
pars.delta_t = delta_t;

% Constraints
u_min = [-1e1; -1e1]; % [N]
u_max = [1e1; 1e1]; % [N m]

pars.u_min = u_min;
pars.u_max = u_max;

% Gain of nominal controller
pars.k = 50;

%% Data-driven controller: model estimator

% Buffer size
pars.estBufferSize = 200;   

% Order of estimation model (dimension of its state vector)
pars.modelOrder = 5;

% Number of samples (# of delta_t's) between model updates
pars.modelUpdDelay = 1;

% Number of learning phases for model
pars.modelLearns = 1e9;

% Initial state-space parameter estimates (Aest0, Best0, Cest0)
estFromSim = 0;

if estFromSim
    % ---- From previous simulations
    load('SS0.mat')
    pars.Aest0 = Aest0;
    pars.Best0 = Best0;
    pars.Cest0 = Cest0;
    pars.Dest0 = zeros(p, l);
else
    % ---- Manual
    sysTmp = rss(pars.modelOrder, p, l);
    sysTmp = c2d(sysTmp, delta_t);
    pars.Aest0 = sysTmp.A;
%     pars.Best0 = sysTmp.B;
    pars.Best0 = [0 0; 0 0; 0 0; 1 0; 0 1];
    pars.Cest0 = sysTmp.C;
%     pars.Cest0 = eye(p);
    pars.Dest0 = zeros(p, l);
    clear sysTmp;
end

%% Data-driven controller: optimal controller

% Optimal controller mode
%     1 - model-predictive control (MPC)
%     2 - MPC with estimated model, a.k.a. adaptive MPC, or AMPC
%     3 - RL/ADP (as stacked Q-learning with horizon N) using true model for prediction
%     4 - RL/ADP (as stacked Q-learning with horizon N) using estimated model for prediction 
%     5 - RL/ADP (as N-step roll-out Q-learning) using true model for prediction
%     6 - RL/ADP (as N-step roll-out Q-learning) using estimated model for prediction 
% 
%     If N = 1, methods are model-free
pars.optCtrlMode = 6;

% Discounting factor
pars.gamma = 1;

% Control horizon length (actor)
pars.ctrlStackSize = 30;

% Critic buffer size, SHOULD BE BETWEEN NcriticPars AND estBufferSize!
pars.criticStackSize = 50;

% Number of samples (# of delta_t's) between critic updates
pars.criticUpdDelay = 5;

% Number of learning phases for critic
pars.criticLearns = 1e9;

% Parameters of running cost
%   r( y , u ) = 1/2 y' S y + 1/2 u' R u
pars.rcostS = diag( [10 10 1 0 0] );
pars.rcostR = diag( [0 0] );

% Probing noise switch and power
pars.isProbNoise = 0;
pars.probNoisePow = 5;

% Critic structure
pars.critStruct = 3;

% Number of critic parameters. Features must include mixed terms for data-driven Q-learning with N = 1
if pars.critStruct == 1 % Quadratic-linear approximator
    pars.NcriticPars = ( (p+l) + 1 ) * (p+l) /2 + (p+l); 
    pars.Wmin = repmat(-1e3, pars.NcriticPars, 1);
    pars.Wmax = repmat(1e3, pars.NcriticPars, 1);
 
elseif pars.critStruct == 2 % Quadratic approximator
    pars.NcriticPars = ( (p+l) + 1 ) * (p+l) /2;
    pars.Wmin = zeros(pars.NcriticPars, 1);
    pars.Wmax = repmat(1e3, pars.NcriticPars, 1);    
    
elseif pars.critStruct == 3 % Quadratic approximator, no mixed terms    
    pars.NcriticPars = p + l;
    pars.Wmin = zeros(pars.NcriticPars, 1);
    pars.Wmax = repmat(1e3, pars.NcriticPars, 1); 
    
elseif pars.critStruct == 4 % W(1) y(1)^2 + ... W(p) y(p)^2 + W(p+1) y(1) u(1) + ... W(...) u(1)^2 + ... 
    pars.NcriticPars = p + p*l +l;
    pars.Wmin = repmat(-1e3, pars.NcriticPars, 1);
    pars.Wmax = repmat(1e3, pars.NcriticPars, 1);    
    
end

% Initial critic weights
criticFromSim = 0;
if criticFromSim
    % ---- From previous simulations
    load('W0.mat');
    pars.W0 = W0;
else
    % ---- Manual
    pars.W0 = rand(pars.NcriticPars, 1);    % <- subject to tuning. Random initialization is a choice
end