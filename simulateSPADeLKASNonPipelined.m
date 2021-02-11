function [time,time_u,yL,df,MSE,ST] = simulateSPADeLKASNonPipelined(h,tauSystemScenarios,phi,Gamma,C_aug,K,F,pattern,SIMULATION_TIME,fh,reference,SYSTEM_MODEL)
% SIMULATESPADENONPIPELINED - A function to simulate the LQR SPADe design
% in Matlab for non-pipelined implementation. The reference change only
% changes the initial state disturbance
% Arguments:
%       h, tauSystemScenarios: Array of 'h' and 'tau' value for the scenarios
%       phi, Gamma, C_aug: Array of state-space matrices for the corresponding tauSystemScenarios
%       K, F: Feedback and Feedforward gains for the corresponding tauSystemScenarios
%       pattern: An ordered cell array of the workload patterns to simulate
%       SIMULATION_TIME: The total simulation time (to plot) in seconds
%       fh: fh=1/FRAME_RATE, where FRAME_RATE is the camera frame rate
%       reference: reference value in array form or a single value
%       SYSTEM_MODEL: which system model to choose in systemModel.m
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
% Dependencies: systemModel.m --> loads the state-space matrices
%               expressionToTimingPattern.m --> converts the pattern for simulation
%               plotPublication.m --> publication-ready plots
% Assumptions: 1) SISO feedback system where the third state is controlled.
%                 Reference changes are hardcoded to the third state x0(3) and
%                 z0(3) in this code.
%              2) Simulation is for LQR controller
% Adaptations: 1) To include/remove feedforward gain, change u=Kz <-> u=Kz+Fr
%              2) To simulate LQI, you need to augment an error state, i.e.
%                 u=K*[z;e], where e is the error state. During control
%                 design the state-space matrices need to be augmented as well.
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
%% LOAD THE SYSTEM MODEL
[A,~,~,~]=systemModel(SYSTEM_MODEL);
nSimulationSteps=ceil(SIMULATION_TIME/fh);
if length(reference)< nSimulationSteps
    %% resizing the reference variable so that length(reference)=nSimulationSteps
    reference=[reference reference(length(reference))*ones(1,nSimulationSteps-length(reference))];
end
initialStateDisturbance=reference(1); %HARDCODED to z0(3) in line 81
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
          z0(3)=initialStateDisturbance;
        end        
      %% simulating LQR controllers with switching
        if i==trackCurrentPeriod
            m=timing_pattern{loop}(j);
            y(timeScenario) = C_aug{m}*z0;  
            u(timeScenario) = K{m}*z0;
%             u(timeScenario) = K{m}*z0+F{m}*reference(i);
            z_1 = phi{m}*z0 + Gamma{m}*u(timeScenario);
            z0 = z_1;
            trackCurrentPeriod=trackCurrentPeriod+(h(m)/fh);
            if i>1
                timeY(timeScenario) = timeY(timeScenario-1) + h(m);
                timeU(timeScenario) = timeY(timeScenario-1) + tauSystemScenarios(m); 
            end
            timeScenario=timeScenario+1;  
            if j==length(timing_pattern{loop})
                j=1;
            else
                j=j+1;
            end
        end
    end 
    %% Storing values needed for plotting
    time{loop}=timeY;
    yL{loop}=y;
    time_u{loop}=timeU;
    df{loop}=u;
    mse_r{loop}=reference(1:length(yL{loop}));
    %% Compute MSE
    MSE(loop)=immse(yL{loop},mse_r{loop});
    %% Compute ST
    st = stepinfo(yL{loop},time{loop},reference(1),'SettlingTimeThreshold',0.05);
    ST(loop) = st.SettlingTime;
    clear timeY timeU u y x0 z0   
end
%%END: iterate for each pattern
%% Plot results: yL output, df output, MSE, ST
%BEGIN:ADD ^\omega to pattern for legends in the fig
for i=1:length(pattern)
    if isa(pattern{i},'char')
        pattern{i}=sprintf('(%s)^{\\omega}',pattern{i});
    else
        pattern{i}=sprintf('[%s]^{\\omega}',num2str(pattern{i}));
    end
end
%END:ADD ^\omega
%% plot time vs yL
s = rng; %random seed to have same colours for both the plots. E.g. rng(1,'twister');
plotPublication(pattern,time,yL,'time (s)', 'yL (m)', 'yL',s); 
%% plot time vs df
rng(s); %setting the same random seed to have same colours for the second plot
plotPublication(pattern,time_u,df,'time (s)', '\delta_f (radians)', '\delta_f',s); 
%% plot MSE
figure('name', 'Mean Square Error')    
bar(MSE);
set(gca,'xticklabel',pattern);
%% plot ST
figure('name', 'Settling Time');    
bar(ST);
set(gca,'xticklabel',pattern);
%% Save the workspace
%save('plot.mat','time','yL','df');
fprintf('===================================================\n');

