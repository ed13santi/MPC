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
param.Ts = 0.05;

% This is a sample way to send reference points
param.xTar = shape.target(1);
param.yTar = shape.target(2);

% Horizon length
param.N = 15;

% How much before Ts to contraint final position
param.advance = 0.95;

% Load model parameters and calculate matrices
param.craneParams = load('Crane_NominalParameters.mat');

end % End of mySetup






