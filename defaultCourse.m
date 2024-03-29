function [ course ] = defaultCourse( dis, coursenum )
%DEFAULTCOURSE Generate the default course for the core coursework

if coursenum == 1
    % The shape for the first part
    constraints.rect = [0.00, 0.05;
                        0.45, 0.50;
                        0.50, 0.45;
                        0.05, 0.00];
    
    %constraints.rect = [0.000, 0.001;
    %                    0.399, 0.400;
    %                    0.400, 0.399;
    %                    0.001, 0.000];
                    
    constraints.ellipses = {};
    
    penalties = [-1, -2, -1, -1];
    
    start  = [0.05, 0.05];
    target = [0.4, 0.4];
    
    Wmax = 1;
    Tf   = 5;
elseif coursenum == 2
    % The shape for the second part
    constraints.rect = [0.00, 0.05;
                        0.25, 0.30;
                        0.50, 0.05;
                        0.25, -0.20];

    ellipse.a  = 0.3;
    ellipse.b  = 0.3;
    ellipse.xc = 0.25;
    ellipse.yc = -0.20;

    constraints.ellipses{1} = ellipse;

    penalties = [-1, -1, -1, -1, -1];
    
    start  = [0.05, 0.05];
    target = [0.45, 0.05];
    
    Wmax = 1;
    Tf   = 5;
end

shape.constraints = constraints;

tolerances.state = [ 0.02;   % Cart x position
                     0.02;   % Cart y position
                     0.02;   % Cart x velocity
                     0.02;   % Cart y velocity
                     2*pi;   % Theta position
                     0.02;   % Theta velocity
                     2*pi;   % Phi position
                     0.02;   % Phi velocity
                     0.02;   % Payload x position
                     0.02 ]; % Payload y position
tolerances.input = [ 0.02;   % X input
                     0.02 ]; % Y input

shape.tolerances  = tolerances;
shape.eps_r       = 0.02;
shape.eps_t       = 0.02;
shape.start       = start;
shape.target      = target;
shape.Wmax        = Wmax;
shape.Tf          = Tf;

% The default shape is always considered problem 1
course.prob      = 1;
course.shape     = shape;
course.penalties = penalties;
course.completed = 0;

course.perturbSize = 0.1;

% Create the perturbations
rng(15012);
randPerturb = @() ( ( rand()-0.5 ) * course.perturbSize ) + 1;

course.perturb.m  = randPerturb();
course.perturb.M  = randPerturb();
course.perturb.MR = randPerturb();
course.perturb.r  = randPerturb();
course.perturb.Tl = randPerturb();
course.perturb.Tx = randPerturb();
course.perturb.Ty = randPerturb();
course.perturb.Vl = randPerturb();
course.perturb.Vx = randPerturb();
course.perturb.Vy = randPerturb();

end
