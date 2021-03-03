function Aeq = leftEquality(N)
    selectLandLDot = [zeros(1,11), 1, 0; 
                      zeros(1,11), 0, 1];
    constraintLandLDots = kron(eye(N), selectLandLDot);
    Aeq = blkdiag(eye(10),constraintLandLDots);
end

