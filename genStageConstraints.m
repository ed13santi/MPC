function [Dt,Et,bt] = genStageConstraints(A,B,D,E,cl,ch,ul,uh) 
%modified version, the input constraints are now ul <= Eu <= uh
    lower = zeros(length(ul), size(E,2));
    Dt = [D*A; -D*A; zeros(length(uh) + length(ul),size(A,2))];
    Et = [D*B; -D*B; E; lower];
    for i=1:length(ul)
        lower(i,i) = -1;
    end
    bt = [ch; -cl; uh; -ul];
end

