function wrefInitial = plannedPath2wrefInitial(path)
    wrefInitial = []
    for i=1:size(path,2)-1
        wrefInitial = [wrefInitial; path(1,i); 0; path(2,i); 0; 0; 0; 0; 0; 0; 0];
    end
    wrefInitial = [wrefInitial; path(1,end); 0; path(2,end); 0; 0; 0; 0; 0];
end

