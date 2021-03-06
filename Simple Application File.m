clc;
clear;
close all;

%% Problem Definition

problem.CostFunction = @(x) Sphere(x);      % Cost Function

problem.nVar = 5;                           % Number of unknown (decision) variables

problem.VarMin = -10;                       % Lower Bound of decision variables
problem.VarMax = 10;                        % Upper Bound of decision variables


%% Parameters of PSO

params.MaxIt = 500;                       % Max no. of Iterations

params.nPop = 50;                          % Population size (swarm size)

params.w = 1;                              % Inertia Coefficient
params.wdamp = 0.99;                       % Damping ratio of Inertia Coefficient
params.c1=2;                               % Personal Acceleration coefficient
params.c2=2;                               % Social Acceleration coefficient

params.ShowIterInfo = true;                % show iteration flag

%% Calling PSO

out = PSO(problem, params);

BestSol = out.BestSol;
BestCosts = out.BestCosts;

%% Results

figure;
%plot(BestCosts, 'LineWidth', 2);
semilogy(BestCosts, 'LineWidth', 2);
xlabel('Iteration');
ylabel('Best Cost');
grid on;

