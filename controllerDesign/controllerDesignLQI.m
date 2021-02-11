function [K,F,cqlf_Ai] = controllerDesignLQI(phi,Gamma,C_aug,Q,R)
% CONTROLLERDESIGNLQI - A function to design LQI controllers using dare() with
%                    controllability decomposition, if uncontrollable.
%   Arguments:
%       phi, Gamma, C_aug: Array of (augmented) state-space matrices
%       Q: tuning parameter Q for 'dare()'.
%       R: tuning parameter R for dare()
%   Returns:
%       K,F :Feedback and Feedforward gains for the corresponding state-space matrices.
%       cqlf_Ai: phi+Gamma*K. Used for checking stability of the switched
%                system
%   Usage:
%       CONTROLLERDESIGNLQI(phi,Gamma,C_aug)
%       CONTROLLERDESIGNLQI(phi,Gamma,C_aug,Q)
%       CONTROLLERDESIGNLQI(phi,Gamma,C_aug,Q,R)
%
% Author: Sajid Mohamed

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Default argument values
if nargin < 3
    error('Not enough input arguments for simulation. For details - type >>help controllerDesign');
end
if nargin < 4
    Q=1;
end
if nargin < 5
    R=1; 
end
%% Design per scenario
fprintf('LQI Controller Design for each scenario\n')
for i=1:size(phi,2)    
    %% Initialise
    n0=length(phi{i});
    %% augmentation for the integral tracking action, if using LQI
    phi{i} = [phi{i}  zeros(n0,1); 
                C_aug{i}        1];
    Gamma{i} = [Gamma{i}; 0];
    C_aug{i} = [C_aug{i} 0];
    n=length(phi{i});
    %% check for controllability
    [phi_ctr,Gamma_ctr,C_ctr,T,k] = ctrbf(phi{i},Gamma{i},C_aug{i});
    no_uncontrollable_states = length(phi_ctr)-sum(k);
    %% Check and set Q
    if isequal(size(Q), [1 1])
        Q = Q*eye(n - no_uncontrollable_states);        
    elseif ~isequal(size(Q), [(n - no_uncontrollable_states) (n - no_uncontrollable_states)])
        error("Tuning Matrix Q is of incorrect dimension.\n Q should be a square matrix of order=%d\n Maybe hardcode Q in controllerDesign.m",n - no_uncontrollable_states);
    end        
    if no_uncontrollable_states > 0
        fprintf('\tScenario s_%d is Uncontrollable and needs decomposition\n',i);
        phi_controlled = phi_ctr(no_uncontrollable_states+1:n, no_uncontrollable_states+1:n);
        Gamma_controlled = Gamma_ctr(no_uncontrollable_states+1:n);
        C_controlled = C_ctr(no_uncontrollable_states+1:n);
        %% Design using dare
        [~,~,G] = dare(phi_controlled, Gamma_controlled, Q,R);
        K_controlled = -G;
        F{i} = 1/(C_controlled*inv(eye(n - no_uncontrollable_states)-(phi_controlled+Gamma_controlled*K_controlled))*Gamma_controlled);
        K_T=[zeros(1,no_uncontrollable_states) K_controlled];
        %% Controller Gain
        K{i} = K_T*T;
    else
        fprintf('\tScenario s_%d is Controllable\n',i); 
        [~,~,G] = dare(phi{i}, Gamma{i}, Q, R);
        K{i} = -G;        
        F{i} = 1/(C_aug{i}*inv(eye(n)-(phi{i}+Gamma{i}*K{i}))*Gamma{i});
    end     
    clear phi_ctr Gamma_ctr C_ctr T k phi_controlled Gamma_controlled C_controlled K_controlled K_T;
end
cqlf_Ai{i}= phi{i} + (Gamma{i}*K{i});
fprintf('===========================================================\n');
