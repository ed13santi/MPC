function u = myMPController(r, x_hat, param)

%% Do not delete this line
% Create the output array of the appropriate size
u = zeros(2,1);
%%

u = param.K*(r - x_hat);

end % End of myMPController
