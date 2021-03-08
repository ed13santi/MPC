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
param.Tf = shape.Tf;    

% This is how to set the sampling interval
param.Ts = 0.1;

% This is a sample way to send reference points
param.xTar = shape.target(1);
param.yTar = shape.target(2);

% This is a sample way to send reference points
initialState = [shape.start(1); 0; shape.start(2); 0; 0; 0; 0; 0];
targetState  = [param.xTar;     0; param.yTar;     0; 0; 0; 0; 0];

% Horizon length
param.N = 15;

% How much before Ts to contraint final position
param.advance = 0.95;

% Load model parameters and calculate matrices
param.craneParams = load('Crane_NominalParameters.mat');

% Create generic state space that can be used at every time step using the
% current values of x and u
[param.genericA, param.genericB, param.modelDerivative] = obtainJacs(param.craneParams);

%Run initial optimisation over whole length of episode using fully
%non-linear model
param.w_guess = runInitialOptimisation(targetState, initialState, param);

figure;
plotx = [];
ploty = [];
for i=1:(length(param.w_guess)-8)/10
   plotx = [plotx param.w_guess(i*10+1)]; 
   ploty = [ploty param.w_guess(i*10+3)]; 
end
scatter(plotx, ploty);

end % End of mySetup

