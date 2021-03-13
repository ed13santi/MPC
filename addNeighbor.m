function out = addNeighbor(el,in)
    out = in;
    if el > 0
        el = max(out, el);
        out = [in; el];
    end
end

