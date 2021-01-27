% CONFIG - CONFIGURATION PARAMETERS FOR SPADe
% Design Parameters:
%   NUM_AVAILABLE_CORES: number of processing cores available n_c^{avl}
%   NUM_PARALLEL_CORES_PER_PIPE: number of cores for parallelisation per pipe
%   FRAME_RATE: Camera frame rate in fps, e.g. 30 fps
%   FD: Inter-Frame Dependence time in s: minimum time between two consecutive start of pipes.
%   TAU_WORKLOAD_SCENARIOS: Array of tau values for each workload scenario
%   Q_MULTIPLIER, R: Controller tuning parameters Q and R.
% Simulation Parameters:
%   SIMULATION_TIME: total simulation time in seconds
%   PATTERN: workload sequence to simulate. Each row is one pattern.
%            Some eg pattern{i} --> 's_2 s_1^{6} s_2', [1], [1 2 3].
%
% Author: Sajid Mohamed

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DESIGN PARAMETERS
NUM_AVAILABLE_CORES = 4;
NUM_PARALLEL_CORES_PER_PIPE = 1;
FRAME_RATE = 60;
FD=0;
TAU_WORKLOAD_SCENARIOS= [25.666 34.316 90.416]/1000;
Q_MULTIPLIER=1; 
R=1;
PRECISION=4;
%% MATLAB SIMULATION PARAMETERS
SIMULATION_TIME=1;
PATTERN={1, ...
         's_2', ...
         's_3', ...
         's_2^{6} s_3^{6} s_1^{12}'
         };