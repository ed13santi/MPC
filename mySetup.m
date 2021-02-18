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

% Set minimum and maximumm horizon length
param.samples_max = 20;
param.samples_min = 10;

% Advance settling time by multiplying Ts by a factor
param.advance = 0.9;

% Cost matrices
param.P = zeros(8);
param.Q = zeros(8);
param.R = eye(2);

% Load model parameters and calculate matrices
load('Crane_NominalParameters.mat');
param.rope_len = r;
[param.A,param.B,param.C,~] = genCraneODE(m,M,MR,r,9.81,Tx,Ty,Vx,Vy,param.Ts);

end % End of mySetup






