function [time,time_u,yL,df,MSE,ST] = simulateSPADeNonPipelined(h,tauSystemScenarios,phi,Gamma,C_aug,K,F,pattern,SIMULATION_TIME,fh,reference,SYSTEM_MODEL,CONTROLLER_TYPE,FEEDFORWARD,REFERENCE_STATE)
% SIMULATESPADENONPIPELINED - A function to simulate the LQR SPADe design in Matlab for non-pipelined implementation
% Arguments:
%       h, tauSystemScenarios: Array of 'h' and 'tau' value for the scenarios
%       phi, Gamma, C_aug: Array of state-space matrices for the corresponding tauSystemScenarios
%       K, F: Feedback and Feedforward gains for the corresponding tauSystemScenarios
%       pattern: An ordered cell array of the workload patterns to simulate
%       SIMULATION_TIME: The total simulation time (to plot) in seconds
%       fh: fh=1/FRAME_RATE, where FRAME_RATE is the camera frame rate
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
%       SYSTEM_MODEL: which system model to choose in systemModel.m
%   Usage:
%       SIMULATESPADENONPIPELINED(h,tauSystemScenarios,phi,Gamma,C_aug,K,pattern)
%       SIMULATESPADENONPIPELINED(h,tauSystemScenarios,phi,Gamma,C_aug,K,F,pattern)
%       SIMULATESPADENONPIPELINED(h,tauSystemScenarios,phi,Gamma,C_aug,K,F,pattern,SIMULATION_TIME)
%       SIMULATESPADENONPIPELINED(h,tauSystemScenarios,phi,Gamma,C_aug,K,F,pattern,SIMULATION_TIME,fh);
%       SIMULATESPADENONPIPELINED(h,tauSystemScenarios,phi,Gamma,C_aug,K,F,pattern,SIMULATION_TIME,fh,reference);
%       SIMULATESPADENONPIPELINED(h,tauSystemScenarios,phi,Gamma,C_aug,K,F,pattern,SIMULATION_TIME,fh,reference,SYSTEM_MODEL);
%       SIMULATESPADENONPIPELINED(h,tauSystemScenarios,phi,Gamma,C_aug,K,F,pattern,SIMULATION_TIME,fh,reference,SYSTEM_MODEL,CONTROLLER_TYPE)
% Dependencies: systemModel.m --> loads the state-space matrices
%               expressionToTimingPattern.m --> converts the pattern for simulation
%               plotPublication.m --> publication-ready plots
% Assumptions: 1) Simulation is for LQR controller
% Adaptations: 1) To include/remove feedforward gain, change u=Kz <-> u=Kz+Fr
% 
% Author: Sajid Mohamed

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Default argument values
if nargin < 7
    error('Not enough input arguments for simulation. For details - type >>help simulateSPADeNonPipelined');
end
if nargin < 8
    F=cell(1,length(K));
end
if nargin < 9
    SIMULATION_TIME=1; %in seconds
end
if nargin < 10
    fh=1/30; %DEFAULT FRAME_RATE = 30 fps
end
if nargin < 11
    reference=-0.03;
end
if nargin < 12
    SYSTEM_MODEL=3;
end
if nargin < 13
    CONTROLLER_TYPE=1;
end
if nargin < 14
    FEEDFORWARD=0;
end
if nargin < 15
    REFERENCE_STATE=3;
end
%% LOAD THE SYSTEM MODEL
[A,~,~,~]=systemModel(SYSTEM_MODEL);
nSimulationSteps=ceil(SIMULATION_TIME/fh);
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
%% BEGIN: Iterate for each pattern
[timing_pattern] = expressionToTimingPattern(pattern,length(tauSystemScenarios));
num_plots=length(timing_pattern);
for loop=1:num_plots
    %% Initialise for every pattern
    fprintf('Simulating Non-Pipelined Implementation: Pattern %d\n',loop);
    for i=1:nSimulationSteps %i progresses at fh
        if i==1 
          %%initialization of variables 
          trackCurrentPeriod=1; %checkSPADe --> to keep track of current h, as i progresses at fh
          timeScenario=1; %tS --> to keep track of time with respect to scenarios
          timeY(1) = 0;
          timeU(1) = 0;
          j=1; %to keep track of the length(timing_pattern[])
          x0 = zeros(length(A),1);  %initialise for the state matrix A
          z0 = [x0; 0]; %augmented state matrix; due to implementation-aware matrices additional zeros are not needed
          e(1) = C_aug{1}*z0- reference(1);
        end
        %% Square reference for LKAS; HARDCODED TIME STEPS
        if (REFERENCE_STATE>0) %% Square reference for LKAS; HARDCODED TIME STEPS
            if (SYSTEM_MODEL==1) || (SYSTEM_MODEL==2) || (SYSTEM_MODEL == 3) %THIS IS SO FOR LKAS
                if i==1 %HARDCODED TIME STEP = 0
                    z0(REFERENCE_STATE)=reference(1);
                elseif i==ceil(2/fh) %HARDCODED TIME STEP = 2 seconds
                    z0(REFERENCE_STATE)=reference(2); %value changed at 2 seconds
                elseif i==ceil(4/fh) %HARDCODED TIME STEP = 4 seconds
                    z0(REFERENCE_STATE)=reference(3); % the value changed at 4 seconds
                end 
            end
        end
      %% simulating LQR controllers with switching
        if i==trackCurrentPeriod
            m=timing_pattern{loop}(j);
            mse_reference(timeScenario)=reference_resized(i);
            y(timeScenario) = C_aug{m}*z0;  
            if CONTROLLER_TYPE==2 %LQI          
                e(timeScenario+1) = e(timeScenario) + y(timeScenario) - mse_reference(timeScenario);
                u(timeScenario) = K{m}*[z0;e(timeScenario)];
            else %LQR
                if FEEDFORWARD==1
                    u(timeScenario) = K{m}*z0+F{m}*mse_reference(timeScenario);
                else
                    u(timeScenario) = K{m}*z0;
                end
            end
            z_1 = phi{m}*z0 + Gamma{m}*u(timeScenario);
            z0 = z_1;
            trackCurrentPeriod=trackCurrentPeriod+(h(m)/fh);
            if i>1 %update simulation times
                timeY(timeScenario) = timeY(timeScenario-1) + h(m);
                timeU(timeScenario) = timeY(timeScenario-1) + tauSystemScenarios(m); 
            end
            timeScenario=timeScenario+1;  
            if j==length(timing_pattern{loop}) %update pattern sequence
                j=1;
            else
                j=j+1;
            end
        end %i==trackCurrentPeriod
    end 
    %% Storing values needed for plotting
    time{loop}=timeY;
    yL{loop}=y;
    time_u{loop}=timeU;
    df{loop}=u;
    mse_r{loop}=mse_reference;
    %% Compute MSE
    MSE(loop)=immse(yL{loop},mse_r{loop});
    %% Compute ST
    st = stepinfo(yL{loop},time{loop},mse_reference(1),'SettlingTimeThreshold',0.05);
    ST(loop) = st.SettlingTime;
    clear timeY timeU u y mse_reference x0 z0   
end
%%END: iterate for each pattern
plotSPADe(pattern,time,time_u,yL,df,MSE,ST);


