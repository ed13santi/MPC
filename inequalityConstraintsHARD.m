function [A, b] = inequalityConstraintsHARD(N, r, tolerances)
    A = zeros(4*N+8*2, 10+13*N);
    b = zeros(4*N+8*2, 1);
%     for i=1:N
%         A(4*i-3,13*i-2) =  1; % u < 1
%         A(4*i-2,13*i-1) =  1; % u < 1
%         A(4*i-1,13*i-2) = -1; % u > 1
%         A(4*i  ,13*i-1) = -1; % u > 1
%         b(4*i-3:4*i-2) =  ones(2,1); % u < 1
%         b(4*i-1:4*i  ) = -ones(2,1); % u > 1
%     end
    A(end-15:end-8, end-9:end-2) = eye(8);
    A(end-7:end, end-9:end-2) = -eye(8);
    b(end-15:end-8) = r + tolerances;
    b(end-7:end) = tolerances - r;
end

