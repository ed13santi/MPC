function r = myTargetGenerator(x_hat, param)
%% Modify the following function for your target generation

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

t_target = t + param.N * param.Ts;
t_reach = param.advance * param.Tf;
r = zeros(8,1);
if t_target >= t_reach
    % Make the crane go to (xTar, yTar)
    r(1,1) = param.xTar;
    r(3,1) = param.yTar;
else
    trgt_index = t_target / param.Ts + 1;
    r = param.w_guess(trgt_index*10-9:trgt_index*10-2);
end

end % End of myTargetGenerator

