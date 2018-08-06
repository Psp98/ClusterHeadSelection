function out = PSO(problem, params)
    
    %% Problem Definition

    CostFunction = problem.CostFunction;            % Cost Function

    nVar = problem.nVar;                            % Number of unknown (decision) variables

    VarSize = [1 nVar];                             % Matrix size of decision variables

    VarMin = problem.VarMin;                        % Lower Bound of decision variables
    VarMax = problem.VarMax;                        % Upper Bound of decision variables


    %% Parameters of PSO

    MaxIt = params.MaxIt;                         % Max no. of Iterations

    nPop = params.nPop;                           % Population size (swarm size)

    w = params.w;                                 % Inertia Coefficient
    wdamp = params.wdamp;                         % Damping ratio of Inertia Coefficient
    c1 = params.c1;                               % Personal Acceleration coefficient
    c2 = params.c2;                               % Social Acceleration coefficient
    
    ShowIterInfo = params.ShowIterInfo;           % The flag for showing iteration info
    
    MaxVelocity = 0.2 * (VarMax - VarMin);
    MinVelocity = -MaxVelocity;


    %% Initialization

    % The particle template
    empty_particle.Position = [];
    empty_particle.Velocity = [];
    empty_particle.Cost = [];
    empty_particle.Best.Position = [];
    empty_particle.Best.Cost = [];

    % Create population array
    particle = repmat(empty_particle, nPop, 1);

    % Initialize Global Best
    GlobalBest.Cost = inf;

    %  Initialize Population members
    for i=1:nPop

        % generate random sol
        particle(i).Position = unifrnd(VarMin, VarMax, VarSize);

        % Initialize velocity
        particle(i).Velocity = zeros(VarSize);

        % Evaluation
        particle(i).Cost = CostFunction(particle(i).Position);

        % update personal best
        particle(i).Best.Position = particle(i).Position;
        particle(i).Best.Cost = particle(i).Cost;

        % Update global best
        if particle(i).Best.Cost < GlobalBest.Cost
            GlobalBest = particle(i).Best;
        end    

    end    

    % array to hold best cost values on each iteration
    BestCosts = zeros(MaxIt, 1);


    %% Main Loop of PSO

    for it=1:MaxIt

        for i=1:nPop

            % update velocity
           particle(i).Velocity = w*particle(i).Velocity + c1*rand(VarSize).*(particle(i).Best.Position - particle(i).Position)+ c2*rand(VarSize).*(GlobalBest.Position - particle(i).Position);

           % Apply velocity limits
           particle(i).Velocity = max(particle(i).Velocity, MinVelocity);
           particle(i).Velocity = min(particle(i).Velocity, MaxVelocity);
           
           % update position
           particle(i).Position = particle(i).Position + particle(i).Velocity;
           
           % Apply lower and upper bound limits
           particle(i).Position = max(particle(i).Position,VarMin);
           particle(i).Position = min(particle(i).Position,VarMax);

           % Evaluation
           particle(i).Cost = CostFunction(particle(i).Position);

           % update personal best
           if particle(i).Cost < particle(i).Best.Cost

               particle(i).Best.Position = particle(i).Position;
               particle(i).Best.Cost = particle(i).Cost;

               % Update global best
               if particle(i).Best.Cost < GlobalBest.Cost
                   GlobalBest = particle(i).Best;
               end  
           end

        end

        % Store best cost values
        BestCosts(it) = GlobalBest.Cost;

        % display Iteration information
        if ShowIterInfo
            disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCosts(it))]);
        end

        % Damping Inertia Coefficient
        w = w * wdamp;

    end    
    
    out.pop = particle;
    out.BestSol = GlobalBest;
    out.BestCosts = BestCosts;
    
end