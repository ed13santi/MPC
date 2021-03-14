function [success,path] = makePath(A,B,C,D,h,start,target,rectConstr,ellConstr) %MAKE SURE START AND TARGET ARE 2 ELEMENT VECTORS
    coords = [A,B,C,D];
    minx = min(coords(1,:));
    miny = min(coords(2,:));
    maxx = max(coords(1,:));
    maxy = max(coords(2,:));
    
    nx = ceil((maxx-minx)/h);
    ny = ceil((maxy-miny)/h);
    
    outRect = zeros(nx, ny);
    corner = [minx; miny];
    
    start_ind = [floor((start(1)-minx)/h)+1; floor((start(2)-miny)/h)+1];
    trgt_ind = [floor((target(1)-minx)/h)+1; floor((target(2)-miny)/h)+1];
    
    for i=1:size(outRect,1)
        for j=1:size(outRect,2)
            x = minx + (i-1) * h + h/2;
            y = miny + (j-1) * h + h/2;
            violate1 = violateRectConstr(rectConstr, x, y);
            violate2 = violateEllConstr(ellConstr, x, y);
            if violate1 | violate2
                outRect(i,j) = -1;
            end
        end
    end
    
    outRect(start_ind(1),start_ind(2)) = 1;
    
    success = true;
    found = false;
    maxIter = 50000;
    iter = 0;
    while not(found) & iter < maxIter
        [outRect, found] = runIterationPathFinding(outRect, trgt_ind);
        iter = iter + 1;
    end
    
    if not(found)
       error = "Path Finding failed, MaxIter reached" 
       success = false;
    end
       
    path = rect2path(outRect, trgt_ind, minx, miny, h);
end

