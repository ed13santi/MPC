function ub = uConstraint(w_len)
    ub = Inf * ones(w_len, 1);
    for i=11:12:w_len
        ub(i:i+1) = ones(2,1);
    end
end





%% non-linear constraints

