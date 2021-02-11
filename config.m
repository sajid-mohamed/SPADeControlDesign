% CONFIG - CONFIGURATION PARAMETERS FOR SPADe
%
% Design Parameters:
%   NUM_AVAILABLE_CORES: number of processing cores available n_c^{avl}
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
%% CHOOSE SYSTEM MODEL
SYSTEM_MODEL = 4;
%% DESIGN PARAMETERS
NUM_AVAILABLE_CORES = 1;
NUM_PARALLEL_CORES_PER_PIPE = 1;
FRAME_RATE = 60;
FD=0;
TAU_WORKLOAD_SCENARIOS= [100]/1000;
Q = diag([0 0 0 0 0 10^15]); % SYSTEM_MODEL=4
% Q=1; % automatically scales it, if you give single digit
R=1;
TOLERANCE=4;
CONTROLLER=2; %1=LQR, 2=LQI
%% MATLAB SIMULATION PARAMETERS
SIMULATION_TIME=5;
% % REFERENCE=[-0.03];
REFERENCE=0.01; % SYSTEM_MODEL=4
% PATTERN={1, ...
%          's_2', ...
%          's_3', ...
%          's_2^{6} s_3^{6} s_1^{12}'
%          };
PATTERN={1};