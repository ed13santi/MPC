function [ param ] = mySetup(shape)
%% Modify the following function for your setup function

% This is a sample static K matrix for the controller
param.K = [2, 0, 0, 0, 0, 0, 0, 0;
           0, 0, 2, 0, 0, 0, 0, 0];

% This is a sample way to send reference points
param.xTar = shape.target(1);
param.yTar = shape.target(2);

% This is how to set the sampling interval
param.Ts = 0.05;


end % End of mySetup






