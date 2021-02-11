function [phi,Gamma,C_aug,tauSystemScenarios] = implementationAwareMatrices(h,tau,SYSTEM_MODEL,tolerance)
% IMPLEMENTATIONAWAREMATRICES - A function to derive implementation-aware
%                               state-space matrices for pipelined implementation. 
%                               The dimensions of all the matrices are made consistent. 
%                               The dimensions are defined considering length(A) and max(nf).
%                               A - State matrix in the state-space equation; nf=ceil(tau/h)
%   Arguments: 
%       h,tau: arrays of h and tau values for the scenarios
%       tolerance: floating point tolerance since there are comparisons
%                  with zero. tolerance=4 for timing values in ms,
%                                      =7  "     "      "    " us & so on.
%       SYSTEM_MODEL: which system model to choose in systemModel.m
%       
%   Returns:
%       phi, Gamma, C_aug : augmented implementation-aware state-space matrices 
%                           in discrete-time domain.
%       tauSystemScenarios: tau values for the systemScenarios considering
%                           tauPrime
%   Usage:
%       IMPLEMENTATIONAWAREMATRICES(h,tau)
%       IMPLEMENTATIONAWAREMATRICES(h,tau,tolerance)
%       IMPLEMENTATIONAWAREMATRICES(h,tau,SYSTEM_MODEL,tolerance)
%
% Author: Sajid Mohamed

% Assumptions: 1) Constant sampling period h for pipelining
% Dependencies: systemModel.m --> to load the state-space system matrices
% Constraints:  1) length(tau) should be equal to length(nf)
%               2) tau cannot be negative
% UNRESOLVED ISSUES: HOW TO SET PRECISION AUTOMATICALLY IN FPRINTF STATEMENTS?
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Default argument values
if nargin < 2
    error('Not enough input arguments for simulation. For details - type >>help implementationAwareMatrices');
end
if nargin < 3
    SYSTEM_MODEL = 3;
end
if nargin < 4
    tolerance=4; 
end
%% LOAD THE SYSTEM MODEL and initialise
[Ac,Bc,Cc,Dc]=systemModel(SYSTEM_MODEL);
sysc = ss(Ac,Bc,Cc,Dc);
n=length(Ac);
nf=max(ceil(tau/h),1);
nf_wc=max(nf);
fprintf('Implementation-Aware matrices with n_{f_{wc}} = %d,',nf_wc);
%% discretise the state-space matrices for constant h
if length(h)==1 %We assume a constant h
    sysd = c2d(sysc, h); As_s = sysd.a; Bd = sysd.b; Cd = sysd.c;
else %length(h) ~=1
    error('In the current implementation, h should be constant for pipelining');
end
%% Compute \tau^{\prime}
tau_Prime = zeros(1,length(nf));
for i=1:length(nf)
    if round(tau(i),tolerance)>=0
        tau_Prime(i)=tau(i)-((nf(i)-1)*h);
    else %% when tau(i) < 0, take care of the tolerance!
        error('The delay for any scenario cannot be negative');
    end
end
tauPrime=max(tau_Prime); %\tau^{\prime} is constant for all scenarios
fprintf('\t tau^prime=%.3f\n',tauPrime);
tauSystemScenarios = zeros(1,length(tau));
%% Implementation-Aware matrices' computation
for i=1:length(nf)+1 
    clear Gamma0 Gamma1;  
    if i<=length(nf)
        tauSystemScenarios(i)=((nf(i)-1)*h)+tauPrime;
        if round(tau(i),tolerance)> 0 %% when tau > 0
            %% check if tau is an integral multiple of h
            if round(tauPrime,tolerance)==round(h,tolerance)
                %% tau is an integral multiple of h and \tau^{\prime}==h
                Gamma0 = [zeros(n,1)];
                Gamma1 = Bd;        
            else
                %% tau is NOT an integral multiple of h, \tau^{\prime} < h
                sysd1 = c2d(sysc, h-tauPrime); 
                Gamma0 = sysd1.b;
                Gamma1 = Bd - Gamma0;
            end             
        else
            %% tau = 0, i.e. there is no delay
            Gamma0 = Bd;
            Gamma1 = [zeros(n,1)];
        end  
        phi2=[zeros(nf_wc-1,n) zeros(nf_wc-1,1)    eye(nf_wc-1, nf_wc-1)];
        phi3=[zeros(1,n)        zeros(1,1)           zeros(1,nf_wc-1)];
        if round(tau(i),tolerance)<=round(h,tolerance)     
            %% augmented system when tau <= h
            fprintf('\tScenario s_%d: for delay tauWorkloadScenario(%d) = %.3f, tauSystemScenario(%d) =%.3f, period: h=%.3f, n_{f_i}=%d --> case: (tau<=h)\n',i,i,tau(i),i,tauSystemScenarios(i),h,nf(i));
            phi{i} = [As_s zeros(n,nf_wc-nf(i)) Gamma1;
                      phi2;
                      phi3];
            Gamma{i} = [Gamma0; 
                        zeros(nf_wc-1,1);
                        1];
            C_aug{i} = [Cd zeros(1,nf_wc)];
        else
            %% augmented system when tau > h
            fprintf('\tScenario s_%d: for delay tauWorkloadScenario(%d) = %.3f, tauSystemScenario(%d) =%.3f, period: h=%.3f, n_{f_i}=%d --> case: (tau>h)\n',i,i,tau(i),i,tauSystemScenarios(i),h,nf(i));
            phi{i} = [As_s	zeros(n,nf_wc-nf(i))	Gamma1	Gamma0	zeros(n,nf(i)-2);
                      phi2;
                      phi3];
            Gamma{i} = [zeros(n+nf_wc-1,1);
                        1];
            C_aug{i} = [Cd zeros(1,nf_wc)];
        end
    else %i==length(nf)+1
        %% Model for simulating the case where tau=0, h=h
        %This is an additional mode to take care of workload variations
        %during pipelining --> case 2: when the latest measurement was
        %already available at the previous control computation task.
        fprintf('\tIntermediate scenario s_%d: for delay tau = 0, period: h=%.3f --> to consider control computations when the latest measurement is not available\n',i,h);
        phi{i} = [As_s zeros(n,nf_wc);
                  phi2;
                  phi3];
        Gamma{i} = [Bd; zeros(nf_wc-1,1);  1];
        C_aug{i} = [Cd zeros(1,nf_wc)];
    end    
end
fprintf('======================================================================================================================================\n');
