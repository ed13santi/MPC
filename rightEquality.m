function beq = rightEquality(N, x0, r)
    LandLDotConstraintSingle = [r;0];
    LandLDotConstraints = kron(ones(N+1,1), LandLDotConstraintSingle);
    beq = [x0; LandLDotConstraints];
end


%% non-linear constraints

