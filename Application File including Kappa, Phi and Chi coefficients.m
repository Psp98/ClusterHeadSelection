clc;
clear;
close all;

%% Problem Definition

problem.CostFunction = @(x) Sphere(x);      % Cost Function

problem.nVar = 10;                           % Number of unknown (decision) variables

problem.VarMin = -10;                       % Lower Bound of decision variables
problem.VarMax = 10;                        % Upper Bound of decision variables


%% Parameters of PSO

kappa = 1;
phi1 = 2.05;
phi2 = 2.05;
phi = phi1 + phi2;
chi = 2*kappa/abs(2-phi-sqrt(phi^2-4*phi));

params.MaxIt = 500;                       % Max no. of Iterations

params.nPop = 50;                          % Population size (swarm size)

params.w = chi;                              % Inertia Coefficient
params.wdamp = 0.99;                       % Damping ratio of Inertia Coefficient
params.c1 = chi*phi1;                               % Personal Acceleration coefficient
params.c2 = chi*phi2;                               % Social Acceleration coefficient

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

