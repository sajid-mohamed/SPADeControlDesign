% MAINSPADE - main file for SPADe Backend Design and Matlab Simulation
%
% Author: Sajid Mohamed (s.mohamed@tue.nl)
%
% HOW TO USE: 1) Define your configuration in config.m
%             2) run mainSPADe
%
% Assumptions: 1) Constant sampling period h for pipelining
%              2) NUM_PARALLEL_CORES_PER_PIPE assumes that there is no
%                 resource sharing between pipes. 
%                 In case, you want to consider resource sharing or mapping
%                 optimisations, hard-code the parameters as explained in 
%                 the "Adaptations" .
%
% Adaptations: 1) HOW TO CONSIDER OPTIMAL MAPPING & RESOURCE SHARING:
%                   --> set NUM_AVAILABLE_CORES = max number of pipes p;
%                   --> set NUM_PARALLEL_CORES_PER_PIPE = 1;
%                   --> set tauWorkloadScenarios = [tau(1) ...], where
%                       tau(i) is for the optimised mapping 
%
% Dependencies: 1) systemModel.m --> to load state-space matrices
%               2) implementationAwareMatrices.m --> to generate dimension
%                       consistent state-space and controller matrices
%               3) controllerDesign.m --> to design LQR controllers for
%                       augmented systems
%               4)5) simulateSPADePipelined.m, simulateSPADeNonPipelined 
%                       --> to simulate the pattern considering workload
%                           variations
%               6) plotPublication.m --> publication-ready plots generation
%               7) expressionToTimingPattern.m --> fn to convert the input
%                       timing pattern in mainSPADe.m for simulation
%               8) augmentSystem.m --> generic function to augment the
%                       state-space matrices for any (tau,h) combination.
%               9) config.m --> (User-defined) input configuration parameters
%
% References: SPADe approach is described in detail in the following publications 
%	[1] S. Mohamed, et al., "A scenario-and platform-aware design flow for image-based control systems", In MICPRO, 2020.
%	[2] S. Mohamed, et al., "Designing image-based control systems considering workload variations", In CDC, 2019.
%   [3] S. Mohamed, et al., "Optimising Quality-of-Control for Data-intensive Multiprocessor Image-Based Control Systems considering Workload Variations", In DSD, 2018.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear variables;
close all;
format short;
% help mainSPADe;
%% LOAD THE DESIGN CONFIGURATION AND SIMULATION PARAMETERS
config;
%% camera and pipelining configuration
fh         = 1/FRAME_RATE;
tau_wc     = max(TAU_WORKLOAD_SCENARIOS); %worst-case delay
nf         = ceil(TAU_WORKLOAD_SCENARIOS/fh); 
nf_wc      = max(nf);     % max #frames for pipelining
ns         = floor(FD/fh);% #frames to skip due to inter-frame dependency.
nc_maxPipe = ceil(nf_wc/(ns+1)); %max #cores needed for full pipelining, i.e. for h=fh
ncParallel = NUM_PARALLEL_CORES_PER_PIPE;
nc_max     = ncParallel*nc_maxPipe;
pipelining = 1; %default: true
%% CHECK THE SCENARIO AND DISPLAY TO THE USER
%% CASE NO PIPELINING POSSIBLE
if NUM_AVAILABLE_CORES <= 1    
    pipelining=0;
    fprintf('Pipelining is NOT possible. You need atleast two cores for pipelining.\n');
    fprintf('======================================================================\n');
elseif nc_maxPipe <= 1 %ceil(tau_wc/fh)
    pipelining=0;
    fprintf('Pipelining is NOT possible. Worst-case delay, camera frame rate or frame dependencies limit pipelining for this configuration.\n');
    fprintf('==============================================================================================================================\n');
elseif NUM_AVAILABLE_CORES < nc_max
    h=ceil(nc_max*(ns+1)/NUM_AVAILABLE_CORES)*fh;
else
    h=(ns+1)*fh;    
end
%% Design & Simulation
if pipelining==1    
    fprintf('\t\t\tSPADe Pipelined Implementation\n');
    %% Pipelined Controller Design
    [phi,Gamma,C_aug,tauSystemScenarios] = implementationAwareMatrices(h,TAU_WORKLOAD_SCENARIOS,TOLERANCE);
    [K,F,cqlf_Ai] = controllerDesign(phi,Gamma,C_aug,Q_MULTIPLIER,R);
    %% Pipelined Controller Simulation considering workload variations
    simulateSPADePipelined(h,tauSystemScenarios,phi,Gamma,C_aug,K,F,PATTERN,SIMULATION_TIME);
else
    h=ceil(TAU_WORKLOAD_SCENARIOS/fh)*fh;
    fprintf('\t\t\tSPADe Non-Pipelined Implementation\n');
    %% Non-Pipelined Controller Design
    [phi,Gamma,C_aug] = augmentSystem(TAU_WORKLOAD_SCENARIOS,h);
    [K,F,cqlf_Ai] = controllerDesign(phi,Gamma,C_aug,Q_MULTIPLIER,R);
    %% Non-Pipelined Controller Simulation
    simulateSPADeNonPipelined(h,TAU_WORKLOAD_SCENARIOS,phi,Gamma,C_aug,K,F,PATTERN,SIMULATION_TIME,fh);
end    

