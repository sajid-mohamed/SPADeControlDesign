function [K,F,cqlf_Ai] = controllerDesignLQR(phi,Gamma,C_aug,Q,R)
% CONTROLLERDESIGNLQR - A function to design LQR controllers using dare() with
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
%       CONTROLLERDESIGNLQR(phi,Gamma,C_aug)
%       CONTROLLERDESIGNLQR(phi,Gamma,C_aug,Q)
%       CONTROLLERDESIGNLQR(phi,Gamma,C_aug,Q,R)

%   Adaptations: 
%       1) If you would like to hardcode Q in this code to change with n: 
%                   a. set Q=1 in config.m; 
%                   b. put your Q in line 50
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
fprintf('LQR Controller Design for each scenario\n')
for i=1:size(phi,2)
    %% Initialise
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
    %% HARDCODING Q TO CHANGE WITH n
    % If you would like to hardcode Q: 1. set Q=1 in config.m; 2) uncomment
    % Q=Q*[zeros(n-1,n-1) zeros(n-1,1);zeros(n-1,1) 10^15]; %AN EXAMPLE
    if no_uncontrollable_states > 0
        fprintf('\tScenario s_%d is Uncontrollable and needs decomposition\n',i);
        phi_controlled = phi_ctr(no_uncontrollable_states+1:n, no_uncontrollable_states+1:n);
        Gamma_controlled = Gamma_ctr(no_uncontrollable_states+1:n);
        C_controlled = C_ctr(no_uncontrollable_states+1:n);  
        %% Design using dare
        [~,~,G] = dare(phi_controlled, Gamma_controlled,Q,R);
        K_controlled = -G;
        %% Augmenting uncontrollable states
        K_T=[zeros(1,no_uncontrollable_states) K_controlled];
        %% Controller Gain
        K{i} = K_T*T;
        %% Feedforward Gain
        F{i} = 1/(C_controlled*inv(eye(n - no_uncontrollable_states)-(phi_controlled+Gamma_controlled*K_controlled))*Gamma_controlled);
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
