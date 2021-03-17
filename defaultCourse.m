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
    target = [0.45, 0.45];
    
    Wmax = 1;
    Tf   = 10;
elseif coursenum == 2
    % The shape for the second part
    constraints.rect = [0.00, 0.05;
                        0.25, 0.30;
                        0.50, 0.05;
                        0.25, -0.20];
    ellipse.a  = 0.2;
    ellipse.b  = 0.4;
    ellipse.xc = 0.25;
    ellipse.yc = -0.20;

    constraints.ellipses{1} = ellipse;
    
    ellipse.a  = 0.03;
    ellipse.b  = 0.03;
    ellipse.xc = 0.07;
    ellipse.yc = 0.1;

    constraints.ellipses{2} = ellipse;
    
    ellipse.a  = 0.03;
    ellipse.b  = 0.03;
    ellipse.xc = 0.43;
    ellipse.yc = 0.1;

    constraints.ellipses{3} = ellipse;
    
    ellipse.a  = 0.03;
    ellipse.b  = 0.03;
    ellipse.xc = 0.25;
    ellipse.yc = 0.25;

    constraints.ellipses{4} = ellipse;
    
    

    penalties = [-1, -1, -1, -1, -1, -1, -1, -1];
    
    start  = [0.05, 0.05];
    target = [0.45, 0.05];
    
    Wmax = 1;
    Tf   = 15;
    
elseif coursenum==3
    constraints.rect=[-0.75,-0.75;0.75,-0.75;0.75,0.75;-0.75,0.75]

    ellipse0.xc=0.0;
    ellipse0.yc=0.0;
    ellipse0.a=0.25;
    ellipse0.b=0.25;

    ellipse1.xc=0.2;
    ellipse1.yc=0.2;
    ellipse1.a=0.05;
    ellipse1.b=0.75;

    ellipse2.xc=-0.2;
    ellipse2.yc=-0.7;
    ellipse2.a=0.05;
    ellipse2.b=0.35;

    ellipse3.xc=0.0;
    ellipse3.yc=0.0;
    ellipse3.a=0.6;
    ellipse3.b=0.05;
    constraints.ellipses={ellipse0,ellipse1,ellipse2,ellipse3};
    penalties = [-1, -1, -1, -1, -1, -1, -1, -1];
    start=[0, 0.6];
    target=[0.5, 0.5];
    
    
    
    Wmax = 1;
    Tf   = 15;
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
