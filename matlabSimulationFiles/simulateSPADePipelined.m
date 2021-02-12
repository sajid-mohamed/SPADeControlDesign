function [time,time_u,yL,df,MSE,ST] = simulateSPADePipelined(h,tauSystemScenarios,phi,Gamma,C_aug,K,F,pattern,SIMULATION_TIME,reference,SYSTEM_MODEL,CONTROLLER_TYPE,FEEDFORWARD,REFERENCE_STATE)
% SIMULATESPADEPIPELINED - A function to simulate the SPADe design in Matlab for pipelined implementation
% Arguments:
%       h: the constant sampling period
%       tauSystemScenarios: Array of tau value for the scenarios
%       phi, Gamma, C_aug: Array of state-space matrices for the corresponding tauSystemScenarios
%       K, F: Feedback and Feedforward gains for the corresponding tauSystemScenarios
%       pattern: An ordered cell array of the workload patterns to simulate
%       SIMULATION_TIME: The total simulation time (to plot) in seconds
%       reference: reference value in array form or a single value
%       SYSTEM_MODEL: which system model to choose in systemModel.m
%       CONTROLLER_TYPE: currently supports 1: LQR (DEFAULT) and 2: LQI
%       FEEDFORWARD: 1: simulate with feedforward gain (DEFAULT) and 0: simulate without feedforward gain
%       REFERENCE_STATE: 0: reference change is through typical reference (DEFAULT) 
%                        1-length(A): reference changes the state value of
%                        the system model A
% Returns:
%       time, time_u: Arrays that store the x-axis time values for output y
%                     and input u respectively.
%       yL: the array of output values at 'time'
%       df: the array of input values at 'time_u'. df is the steering angle
%           for LKAS model
%       MSE, ST: Mean Squared Error and Settling time for the different patterns
%   Usage:
%       SIMULATESPADEPIPELINED(h,tauSystemScenarios,phi,Gamma,C_aug,K,pattern)
%       SIMULATESPADEPIPELINED(h,tauSystemScenarios,phi,Gamma,C_aug,K,F,pattern)
%       SIMULATESPADEPIPELINED(h,tauSystemScenarios,phi,Gamma,C_aug,K,F,pattern,SIMULATION_TIME) 
%       SIMULATESPADEPIPELINED(h,tauSystemScenarios,phi,Gamma,C_aug,K,F,pattern,SIMULATION_TIME,reference)
%       SIMULATESPADEPIPELINED(h,tauSystemScenarios,phi,Gamma,C_aug,K,F,pattern,SIMULATION_TIME,reference,SYSTEM_MODEL)
%       SIMULATESPADEPIPELINED(h,tauSystemScenarios,phi,Gamma,C_aug,K,F,pattern,SIMULATION_TIME,reference,SYSTEM_MODEL,CONTROLLER_TYPE)
% Dependencies: systemModel.m --> loads the state-space matrices
%               expressionToTimingPattern.m --> converts the pattern for simulation
%               plotPublication.m --> publication-ready plots
% Assumptions: 1) Constant sampling period h
%              2) Simulation is for LQR controller
% Adaptations: 1) To include/remove feedforward gain, change u=Kz <-> u=Kz+Fr
%              2) To simulate LQI, you need to augment an error state, i.e.
%                 u=K*[z;e], where e is the error state. During control
%                 design the state-space matrices need to be augmented as well.
% 
%   Author: Sajid Mohamed (s.mohamed@tue.nl)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Default argument values
if nargin < 7
    error('Not enough input arguments for simulation. For details - type >>help simulateSPADePipelined');
end
if nargin < 8
    F=cell(1,length(K));
end
if nargin < 9
    SIMULATION_TIME=1; %in seconds
end
if nargin < 10
    reference=-0.03;
end
if nargin < 11
    SYSTEM_MODEL=3;
end
if nargin < 12
    CONTROLLER_TYPE=1;
end
if nargin < 13
    FEEDFORWARD=0;
end
if nargin < 14
    REFERENCE_STATE=3;
end
%% LOAD THE SYSTEM MODEL
[A,~,~,~]=systemModel(SYSTEM_MODEL);
nf=max(ceil(tauSystemScenarios/h),1);
nSimulationSteps=ceil(SIMULATION_TIME/h);
if (REFERENCE_STATE > size(A,1))
    error("Undefined System State %d.\n REFERENCE_STATE can take only take values from 1 to %d\n",REFERENCE_STATE,length(A));
elseif (REFERENCE_STATE~=0) && (FEEDFORWARD == 1)
    error("REFERENCE_STATE should be equal to 0 when FEEDFORWARD is 1.\n");
else
end
%% resizing the reference variable so that length(reference)=nSimulationSteps
if REFERENCE_STATE > 0
    reference_resized=zeros(1,nSimulationSteps);
else
    reference_resized=[reference reference(length(reference))*ones(1,nSimulationSteps-length(reference))];
end
if length(reference)< 3
    reference=[reference zeros(1,3-length(reference))];
end
tauPrime=tauSystemScenarios(1)-((nf(1)-1)*h);
%% BEGIN: Iterate for each pattern
[timing_pattern] = expressionToTimingPattern(pattern,length(tauSystemScenarios));
num_plots=length(timing_pattern);
simulatePattern=cell(size(timing_pattern));
for loop=1:num_plots
    %% Initialise for every pattern
    movingWindow=max(nf)*ones(1,max(nf));
    simulatePattern{loop}=max(nf)*ones(size(timing_pattern{loop}));
    timeY=zeros(1,nSimulationSteps);
    timeU=zeros(1,nSimulationSteps);
    u=zeros(1,nSimulationSteps);
    uApplied=zeros(1,nSimulationSteps+max(nf));
    fprintf('Simulating Pipelined Implementation: Pattern %d\n',loop);
    %% Find the effective simulation ordering for the input workload pattern
    % Here, we are looking ahead since we need to compute z0 similarly. 
    % In actual implementation, we will be looking back when control computation task is called.
    for i=1:length(timing_pattern{loop}) %the ordering needs to be found considering the moving window based on max(nf)
        m=timing_pattern{loop}(i);
        if length(timing_pattern{loop})==1
            simulatePattern{loop}(i)=nf(m);
        else %code acts similar to a filter: max(nf) timing_pattern elements are considered to generate one simulatePattern element 
            movingWindow(i)=nf(m);
            simulatePattern{loop}(i)=nf(m);
            for j=1:max(nf)-1 % max(nf) always greater than 1 in pipelining
                if i+j>length(timing_pattern{loop})
                    mj=mod(i+j,length(timing_pattern{loop}));
                    m=timing_pattern{loop}(mj);
                else
                    m=timing_pattern{loop}(i+j);
                end
                movingWindow(i+j)=nf(m);
                if (simulatePattern{loop}(i))-j >= movingWindow(i+j)
                    simulatePattern{loop}(i)=0; %indicates that at this time-step i, latest measurement not available
                    break;
                end
            end            
        end
    end
    %simulatePattern --> captures the effective simulation ordering
    %% Start Simulation for each pattern
    for i=1:nSimulationSteps %i progresses @rate=fh
      if i==1 
          %%initialization of variables        
          firstFrameProcessed=0;
          timeY(1) = 0;
          timeU(1) = 0;
          j=1; %to keep track of the length(timing_pattern[])
          x0 = zeros(length(A),1);  %initialise for the state matrix A
          z0 = [x0; zeros(max(nf),1)]; %augmented state matrix
          nfScenario=simulatePattern{loop}(1);
          e(1) = C_aug{1}*z0- reference(1);
      else
          %%Update timing
          timeY(i) = timeY(i-1) + h;
          if i==2 %% to simulate delay correctly
              timeU(2) = tauPrime;
          else
              timeU(i) = timeU(i-1) + h; 
          end
      end
        %% Square reference for LKAS; HARDCODED TIME STEPS
        if (REFERENCE_STATE>0) %% Square reference for LKAS; HARDCODED TIME STEPS
            if (SYSTEM_MODEL==1) || (SYSTEM_MODEL==2) || (SYSTEM_MODEL == 3) %THIS IS SO FOR LKAS
                if i==1 %HARDCODED TIME STEP = 0
                    z0(REFERENCE_STATE)=reference(1);
                elseif i==ceil(2/h) %HARDCODED TIME STEP = 2 seconds
                    z0(REFERENCE_STATE)=reference(2); %value changed at 2 seconds
                elseif i==ceil(4/h) %HARDCODED TIME STEP = 4 seconds
                    z0(REFERENCE_STATE)=reference(3); % the value changed at 4 seconds
                end 
            end
        end
      %%System scenario @timestep i
      nfPrevScenario=nfScenario;
      nfScenario=simulatePattern{loop}(j);
      systemScenario=timing_pattern{loop}(j); %the system scenario mode to simulate from the pattern. 0 - latest measurement not available
      if nfScenario==0 %check which mode to simulate: 0 means tau=0
          if i==1 %the first simulation step
            u(i)=0;
            uApplied(i)=0;
          else
            u(i)=u(i-1); % In our implementation, if no new measurements are available, the control input is held as the previous.
            uApplied(i+nfPrevScenario+1)=uApplied(i+nfPrevScenario); %for plotting when it is actually applied
          end
          systemScenario=length(phi); %currently the length(phi) controller simulates tau=0,h
      else
          firstFrameProcessed=1;
          if CONTROLLER_TYPE==2 %LQI
              u(i) = K{systemScenario}*[z0;e(i)];
          else %LQR
              if FEEDFORWARD == 1
                  u(i) = K{systemScenario}*z0+F{systemScenario}*reference_resized(i);
              else
                  u(i) = K{systemScenario}*z0;
              end
          end
          uApplied(i+nfScenario)=u(i);
      end
      if firstFrameProcessed==1 %Matrices evolve only after the first frame is processed
          y(i) = C_aug{systemScenario}*z0;
          z_1 = phi{systemScenario}*z0 + Gamma{systemScenario}*u(i);   
          z0 =z_1;    
          if CONTROLLER_TYPE==2         
            e(i+1) = e(i) + y(i) - reference_resized(i);
          end
      else
          y(i)=C_aug{systemScenario}*z0;
          uApplied(i)=0;
      end
      if j==length(timing_pattern{loop}) %iterating the timing pattern index j
          j=1;
      else
          j=j+1;
      end      
    end
    %% Adapting to keep uApplied dimension consistent with timeU
    uApplied=uApplied(1:nSimulationSteps);
    %% Storing values needed for plotting
    time{loop}=timeY;
    yL{loop}=y;
    time_u{loop}=timeU;
    dfCompute{loop}=u;
    df{loop}=uApplied;
    mse_r{loop}=reference_resized(1:length(yL{loop}));
    %% Compute MSE
    MSE(loop)=immse(yL{loop},mse_r{loop});
    %% Compute ST
    st = stepinfo(yL{loop},time{loop},reference(1),'SettlingTimeThreshold',0.05);
    ST(loop) = st.SettlingTime;
    clear timeY timeU timeUApplied u uApplied y x0 z0   
end
%%END: iterate for each pattern
plotSPADe(pattern,time,time_u,yL,df,MSE,ST);