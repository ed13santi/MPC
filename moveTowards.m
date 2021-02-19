function A_moved = moveTowards(A,B,C,eps)
    AB = B - A;
    AC = C - A;
    AB = eps.*AB./norm(AB);
    AC = eps.*AC./norm(AC);
    A_moved = A + AB + AC;
end

