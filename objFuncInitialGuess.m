function out = objFuncInitialGuess(w, N)
    penalties = zeros(8+10*N,1);
    for i=1:N
       penalties(10*i-1:10*i) = ones(2,1); 
    end
    out = w' * diag(penalties) * w;
%     vels = zeros(2*(N+1), 1);
%     us = zeros(2*N, 1);
%     for i = 1:N
%         vels(2*i-1:2*i, 1) = w(13*i-11:2:13*i-9);
%         us(2*i-1:2*i, 1) = w(13*i-2:13*i-1);
%     end
%     vels(2*(N+1)-1:2*(N+1), 1) = w(13*(N+1)-11:2:13*(N+1)-9);
%     
%     vels = vels(3:end,1) + vels(1:end-2,1);
%     prods = us .* vels;
%     maxed = max(prods,[],2);
%     out = sum(maxed);
end




%% linear inequality constraints
