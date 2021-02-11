function simulateSPADe(pipelined,CONTROLLER,h,TAU_WORKLOAD_SCENARIOS,phi,Gamma,C_aug,K,F,PATTERN,SIMULATION_TIME,REFERENCE,SYSTEM_MODEL,fh)
%% Choose the correct simulation
if pipelined==1
    if CONTROLLER == 2 %LQI
        simulateSPADePipelinedLQI(h,TAU_WORKLOAD_SCENARIOS,phi,Gamma,C_aug,K,F,PATTERN,SIMULATION_TIME,REFERENCE,SYSTEM_MODEL); 
    else
        %% Pipelined Controller Simulation considering workload variations
        simulateSPADePipelined(h,TAU_WORKLOAD_SCENARIOS,phi,Gamma,C_aug,K,F,PATTERN,SIMULATION_TIME,REFERENCE,SYSTEM_MODEL); 
    end
else
    if CONTROLLER==2 %LQI     
        simulateSPADeNonPipelinedLQI(h,TAU_WORKLOAD_SCENARIOS,phi,Gamma,C_aug,K,F,PATTERN,SIMULATION_TIME,fh,REFERENCE,SYSTEM_MODEL);
    else
        simulateSPADeNonPipelined(h,TAU_WORKLOAD_SCENARIOS,phi,Gamma,C_aug,K,F,PATTERN,SIMULATION_TIME,fh,REFERENCE,SYSTEM_MODEL);
    end
end