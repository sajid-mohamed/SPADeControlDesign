function simulateSPADe(pipelined,CONTROLLER,h,TAU_WORKLOAD_SCENARIOS,phi,Gamma,C_aug,K,F,PATTERN,SIMULATION_TIME,REFERENCE,SYSTEM_MODEL,fh)
%% Choose the correct simulation
if SYSTEM_MODEL== 4 %not LKAS
    if pipelined==1
        if CONTROLLER == 2 %LQI
            simulateSPADePipelinedLQI(h,TAU_WORKLOAD_SCENARIOS,phi,Gamma,C_aug,K,F,PATTERN,SIMULATION_TIME,REFERENCE,SYSTEM_MODEL); 
        else
            %% Pipelined Controller Simulation considering workload variations
            simulateSPADePipelined(h,TAU_WORKLOAD_SCENARIOS,phi,Gamma,C_aug,K,F,PATTERN,SIMULATION_TIME,REFERENCE,SYSTEM_MODEL); 
        end
    else %non-pipelined
        if CONTROLLER==2 %LQI     
            simulateSPADeNonPipelinedLQI(h,TAU_WORKLOAD_SCENARIOS,phi,Gamma,C_aug,K,F,PATTERN,SIMULATION_TIME,fh,REFERENCE,SYSTEM_MODEL);
        else %LQR
            simulateSPADeNonPipelined(h,TAU_WORKLOAD_SCENARIOS,phi,Gamma,C_aug,K,F,PATTERN,SIMULATION_TIME,fh,REFERENCE,SYSTEM_MODEL);
        end
    end
else    %SYSTEM_MODEL 1-3 is for LKAS
    if pipelined==1
        if CONTROLLER == 2 %LQI
            simulateSPADeLKASPipelinedLQI(h,TAU_WORKLOAD_SCENARIOS,phi,Gamma,C_aug,K,F,PATTERN,SIMULATION_TIME,REFERENCE,SYSTEM_MODEL); 
        else
            %% Pipelined Controller Simulation considering workload variations
            simulateSPADeLKASPipelined(h,TAU_WORKLOAD_SCENARIOS,phi,Gamma,C_aug,K,F,PATTERN,SIMULATION_TIME,REFERENCE,SYSTEM_MODEL); 
        end
    else %non-pipelined
        if CONTROLLER==2 %LQI     
            simulateSPADeLKASNonPipelinedLQI(h,TAU_WORKLOAD_SCENARIOS,phi,Gamma,C_aug,K,F,PATTERN,SIMULATION_TIME,fh,REFERENCE,SYSTEM_MODEL);
        else %LQR
            simulateSPADeLKASNonPipelined(h,TAU_WORKLOAD_SCENARIOS,phi,Gamma,C_aug,K,F,PATTERN,SIMULATION_TIME,fh,REFERENCE,SYSTEM_MODEL);
        end
    end
end