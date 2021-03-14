function [ARows, bRows] = rectLimsRowsInitialGuess(rectConstraints)
    [DRect,chRect,clRect] = rectCon(rectConstraints);
    D = [ 0 0 DRect(1,1) 0 DRect(1,2) 0 0 0 0 0; 
          0 0 DRect(2,1) 0 DRect(2,2) 0 0 0 0 0 ];
    ARows = [D; -D];
    bRows = [ chRect;  
              -clRect ];
end


%% find path
% 
% function xs = findPath(finalTrgt, x_hat, N, param)
%     finalTrgt = [finalTrgt(1); finalTrgt(3)];
%     x_hat = [x_hat(1); x_hat(3)];
%     % initial guess x0
%     x0 = x_hat;
%     for i=1:N
%         x0(2*i+1:2*i+2) = (finalTrgt - x_hat) / N * i + x_hat;
%     end
%     
%     % objective function minimises distance between consecutive xs
%     objFunc = @(x) pathObjFunc(x, N);
%     
%     % linear inequality constraint
%     [A, b] = inequalityConstraintsPath(N, param.constraints.rect);
% 
%     % linear equality constraints 
%     [Aeq, beq] = linearEqConstrPath(x_hat, finalTrgt, N);
% 
%     % non-linear constraints
%     nonlcon = @(x) nonLinearConstraintsPath(true, param.constraints.ellipses, x, N);
% 
%     % options
%     options = optimoptions(@fmincon);%,'MaxFunctionEvaluations', 15000, 'MaxIterations', 10000);
% % 
% %     size(x0)
% %     size(A)
% %     size(b)
% %     size(Aeq)
% %     size(beq)
%     
%     % optimisation
%     A = sparse(A);
%     Aeq = sparse(Aeq);
%     xs = fmincon(objFunc,x0,A,b,Aeq,beq,[],[],nonlcon,options);
% end
% 
% function f = pathObjFunc(x, N)
%     T = [ [ zeros(2*N,2), eye(2*N) ];
%           [ zeros(2,2*N), eye(2) ] ];
%     p = eye(2*(N+1));
%     H = p - T'*p' - p*T + T'*p*T;
%     f = x' * H * x;
% end
% 
% function [A, b] = inequalityConstraintsPath(N, rectConstraints)
%     A = zeros(0, 2+2*N);
%     b = zeros(0, 1);
%     
%     for i=1:N-1        
%         % rectangle constraints
%         [A2, b2] = rectLimRowsPath(rectConstraints);
%         
%         % combine matrices
%         A_tmp = [zeros(size(A2,1), i*2), A2, zeros(size(A2,1), 2*N-i*2)];
%         A = [A; A_tmp];
%         b = [b; b2];
%     end
% end
% 
% function [ARows, bRows] = rectLimRowsPath(rectConstraints)
%     [DRect,chRect,clRect] = rectCon(rectConstraints);
%     D = [ DRect(1,1) DRect(1,2); 
%           DRect(2,1) DRect(2,2) ];
%     ARows = [D; -D];
%     bRows = [ chRect;  
%               -clRect ];
% end
% 
% function [Aeq, beq] = linearEqConstrPath(x_first, x_trgt, N)
%     Aeq = [ [eye(2), zeros(2, 2*N)];
%             [zeros(2, 2*N), eye(2)] ];
%     beq = [x_first; x_trgt];
% end
% 
% function [c, ceq] = nonLinearConstraintsPath(ellConstr, ellipses, x_in, N)
%     % inequality constraints for ellipses
%     if ellConstr == true
%         c = [];
%         for i=1:N-1
%             x = x_in(2*i+1);
%             y = x_in(2*i+2);
%             for j=1:size(ellipses,1)
%                 for z=1:size(ellipses,2)
%                     ell = ellipses{j,z};
%                     c = [c; - ellipseEval(x, y, ell.xc, ell.yc, ell.a, ell.b)];
%                 end
%             end
%         end
%     else
%         c = [];
%     end
%     
%     ceq = [];
% end

%% Find path 2

