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

% Load model parameters (NOTE! These are not precise as they are later
% perturbed)
load('Crane_NominalParameters.mat');
% ASSUMES Vx = Vy
[param.A,param.B,param.C,~] = genCraneODE(m,M,MR,r,9.81,Tx,Ty,Vx,param.Ts);


% This is a sample static K matrix for the controller
%param.K = [2, 0, 0, 0, 0, 0, 0, 0;
%           0, 0, 2, 0, 0, 0, 0, 0];

% This is a sample way to send reference points
param.xTar = shape.target(1);
param.yTar = shape.target(2);




end % End of mySetup






