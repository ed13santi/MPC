function r = myTargetGenerator(x_hat, param)
%% Modify the following function for your target generation

% Create the output array of the appropriate size. This vector is passed
% into myMPController, so its size must match what is expected by the code
% there.
r = zeros(8,1);

% current time
persistent t
if isempty(t)
    t = 0;
else
    t = t + param.Ts;
end

% initial x
persistent x_initial
if isempty(x_initial)
    x_initial = x_hat;
end

t_target = t + param.samples_max * param.Ts;
t_reach = param.advance * param.Tf;
delta_x = param.xTar - x_initial(1);
delta_y = param.yTar - x_initial(3);
if t_target >= t_reach
    % Make the crane go to (xTar, yTar)
    r(1,1) = param.xTar;
    r(3,1) = param.yTar;
else
    ratio = t_target / t_reach;
    r(1,1) = x_initial(1) + ratio * delta_x;
    r(3,1) = x_initial(3) + ratio * delta_y;
end

end % End of myTargetGenerator


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Modify the following function for your state estimator (if desired)
