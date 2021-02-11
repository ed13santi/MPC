function [Dt,Et,bt] = genStageConstraints(A,B,D,cl,ch,ul,uh)
    Dt = [D*A; -D*A; zeros(2*length(ul),size(D*A,2))];
    Et = [D*B; -D*B; eye(size(D*B,2)); -eye(size(D*B,2))];
    bt = [ch; -cl; uh; -ul];
end

