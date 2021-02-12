% CONFIG - CONFIGURATION PARAMETERS FOR SPADe
%
% Design Parameters:
%   NUM_AVAILABLE_CORES: number of processing cores available n_c^{avl}
%   NUM_PIPES: requested number of pipes
%   NUM_PARALLEL_CORES_PER_PIPE: number of cores for parallelisation per pipe
%   FRAME_RATE: Camera frame rate in fps, e.g. 30 fps
%   FD: Inter-Frame Dependence time in s: minimum time between two consecutive start of pipes.
%   TAU_WORKLOAD_SCENARIOS: Array of tau values for each workload scenario
%   Q_MULTIPLIER, R: Controller tuning parameters Q and R.
%   TOLERANCE: tolerance for numerical computations in MATLAB (rounding and comparison, ctrbf)
%
% Simulation Parameters:
%   SIMULATION_TIME: total simulation time in seconds
%   PATTERN: workload sequence to simulate. Each row is one pattern.
%            Some eg pattern{i} --> 's_2 s_1^{6} s_2', [1], [1 2 3].
%
% Author: Sajid Mohamed

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% CAMERA FRAME RATE
FRAME_RATE = 60; %in frame rate per second, e.g. 60 fps
%% IMPLEMENTATION CHOICE
NUM_PIPES = 4;
NUM_PARALLEL_CORES_PER_PIPE = 1;
%% APPLICATION MODEL
SYSTEM_MODEL = 4; % 1: VREP, 2: WEBOTS, 3: LKAS from paper (DEFAULT), 4: Suspension Control System
%% PLATFORM PARAMETERS
NUM_AVAILABLE_CORES = 4; 
%% CONTROL DESIGN PARAMETERS
Q = diag([0 0 10^25 0 0 0 0]);   % SYSTEM_MODEL=4, LQR
% Q = diag([0 0 0 0 0 10^15]); % SYSTEM_MODEL=4, LQI
% Q=1; % automatically scales it, if you give single digit
R=1;
TOLERANCE=4;
CONTROLLER_TYPE=1; %1=LQR, 2=LQI
%% PARAMETERS FROM SDF3 ANALYSIS
FD=0; %in seconds
TAU_WORKLOAD_SCENARIOS= [84 52 12]/1000; %in seconds
%% MATLAB SIMULATION PARAMETERS
SIMULATION_TIME=2; %in seconds
% REFERENCE=[-0.03]; %in units of SYSTEM_MODEL
% REFERENCE=[-0.03 0.03 -0.03]; % 3 values for square response in LKAS at interval of 2 and 4 seconds: HARDCODED
REFERENCE=[0 0 0.01]; % SYSTEM_MODEL=4
PATTERN={1, ...
         's_2', ...
         's_3', ...
         's_2^{6} s_3^{6} s_1^{12}'
         };
% PATTERN={1};
FEEDFORWARD=1;