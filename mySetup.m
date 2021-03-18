function [ param ] = mySetup(shape)
%% Modify the following function for your setup function

% Copy course parameters
param.constraints = shape.constraints;
param.tolerances = shape.tolerances; 
param.eps_r = shape.eps_r;      
param.eps_t = shape.eps_t;      
param.start = shape.start;      
param.target = shape.target;     
param.Wmax = shape.Wmax;   

param.Ts = 0.1; % set sampling period
param.TsFactor = 5; % set sampling frequency reduction factor for initial non-linear optimisation
param.Tf = shape.Tf - param.Ts; % set Tf to be slightly less than required
N_initial = ceil(param.Tf / (param.Ts * param.TsFactor));
param.Ts = param.Tf / (N_initial * param.TsFactor);
param.optimiseEvery = 1;
param.nInitialOpimisations = 1;

param.extraDistanceEllipses = 0.001;
param.extraDistanceRectangles = 0.001;
n_ellipses = size(param.constraints.ellipses, 1) * size(param.constraints.ellipses, 2);
param.nSlackVars = 4 + 2*min(2, n_ellipses);

% This is a sample way to send reference points
param.xTar = shape.target(1);
param.yTar = shape.target(2);

% This is a sample way to send reference points
initialState = [shape.start(1); 0; shape.start(2); 0; 0; 0; 0; 0];
targetState  = [param.xTar;     0; param.yTar;     0; 0; 0; 0; 0];

% Horizon length
param.N = 15;

% Load model parameters and calculate matrices
param.craneParams = load('Crane_NominalParameters.mat');

% Create generic state space that can be used at every time step using the
% current values of x and u
[param.genericA, param.genericB, param.modelDerivative] = obtainJacs(param.craneParams);

%Run initial optimisation over whole length of episode using fully
%non-linear model
param.w_guess = runInitialOptimisation(targetState, initialState, param, param.TsFactor, N_initial);
% convert to xXuxXu...xXuxX format
param.w_guess = convertToIncludeSlackVars(param.w_guess, param.nSlackVars);

segLen = 10+param.nSlackVars;
simulationLength = 20;
extraCopies = ceil(simulationLength / param.Ts) + param.N - (length(param.w_guess) - segLen + 2)/segLen + param.TsFactor;
blk = [0;0;param.w_guess(end-7-param.nSlackVars:end-param.nSlackVars);zeros(param.nSlackVars,1)]; % uxX
repBlk = kron(ones(extraCopies,1), blk); % uxXuxXuxXuxXuxXuxXuxX
param.wref = [ param.w_guess; repBlk ]; % xXuxXu...xXuxX

figure;
plotx = [];
ploty = [];
plotxp = [];
plotyp = [];
for i=1:(length(param.w_guess)-8)/segLen+1
   plotx = [plotx param.w_guess((i-1)*segLen+1)]; 
   ploty = [ploty param.w_guess((i-1)*segLen+3)]; 
   plotxp = [plotxp param.w_guess((i-1)*segLen+1)+param.craneParams.r*sin(param.w_guess((i-1)*segLen+5))]; 
   plotyp = [plotyp param.w_guess((i-1)*segLen+3)+param.craneParams.r*sin(param.w_guess((i-1)*segLen+7))];
end
scatter(plotx, ploty);
hold on;
scatter(plotxp, plotyp);
hold off;



end % End of mySetup

