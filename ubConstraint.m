function ub = ubConstraint(w_len, r, tolerances)
    ub = Inf * ones(w_len, 1);
    for i=0:13:w_len-13
        ub(i+11:i+12) = ones(2,1); % u < 1
    end
    ub(w_len-9:w_len-2) = r + tolerances; % final position constraints
end

