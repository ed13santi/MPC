function path = rect2path(rect, trgtInd, minx, miny, h)
    padded  = [zeros(size(rect,1),1), rect, zeros(size(rect,1),1)];
    padded  = [zeros(1,size(padded,2)); padded; zeros(1,size(padded,2))];

    current = trgtInd + [1;1];
    path = [trgtInd];
    finished = false;
    while not(finished)
        currVal = padded(current(1),current(2));
        next = [-1;-1];
        if padded(current(1)+1,current(2)-1) == currVal - 1
            next = current + [1;-1];
        end
        if padded(current(1)+1,current(2)) == currVal - 1
            next = current + [1;0];
        end
        if padded(current(1)+1,current(2)+1) == currVal - 1
            next = current + [1;1];
        end
        if padded(current(1)-1,current(2)-1) == currVal - 1
            next = current + [-1;-1];
        end
        if padded(current(1)-1,current(2)) == currVal - 1
            next = current + [-1;0];
        end
        if padded(current(1)-1,current(2)+1) == currVal - 1
            next = current + [-1;1];
        end
        if padded(current(1),current(2)-1) == currVal - 1
            next = current + [0;-1];
        end
        if padded(current(1),current(2)+1) == currVal - 1
            next = current + [0;1];
        end
        
        path = [path, next - [1;1]];
        if currVal == 2
            finished = true;
        end
        current = next;
    end
    
    for i=1:size(path,2)
        path(1,i) =  minx + (path(1,i)-1) * h;
        path(2,i) =  miny + (path(2,i)-1) * h;
    end
    
    path = flip(path,2);
end

