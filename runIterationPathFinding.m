function [outRect, found] = runIterationPathFinding(outRect, trgt_ind)
    padded  = [zeros(size(outRect,1),1), outRect, zeros(size(outRect,1),1)];
    padded  = [zeros(1,size(padded,2)); padded; zeros(1,size(padded,2))];
    for i=2:size(outRect,1)+1
        for j=2:size(outRect,2)+1
            if padded(i,j) == 0
                neigh = 0;
                neigh = max(padded(i  ,j-1),neigh);
                neigh = max(padded(i  ,j+1),neigh);
                neigh = max(padded(i-1,j-1),neigh);
                neigh = max(padded(i-1,j+1),neigh);
                neigh = max(padded(i-1,j  ),neigh);
                neigh = max(padded(i+1,j-1),neigh);
                neigh = max(padded(i+1,j+1),neigh);
                neigh = max(padded(i+1,j  ),neigh);
                if neigh > 0
                    outRect(i-1,j-1) = neigh + 1;
                end
            end
        end
    end
    found = (outRect(trgt_ind(1),trgt_ind(2)) > 0);
end

