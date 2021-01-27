function [K,F,cqlf_Ai] = controllerDesign(phi,Gamma,C_aug,Q_multiplier,R)
% CONTROLLERDESIGN - A function to design LQR controllers using dare() with
%                    controllability decomposition, if uncontrollable.
%   Arguments:
%       phi, Gamma, C_aug: Array of (augmented) state-space matrices
%       Q_multiplier: helps to compute tuning parameter Q for 'dare()'.
%                     E.g. Q = Q_multiplier*[zeros(n, n) zeros(n,1);zeros(1,n)  10^10];
%       R: tuning parameter R for dare()
%   Returns:
%       K,F :Feedback and Feedforward gains for the corresponding state-space matrices.
%       cqlf_Ai: phi+Gamma*K. Used for checking stability of the switched
%                system
%   Usage:
%       CONTROLLERDESIGN(phi,Gamma,C_aug)
%       CONTROLLERDESIGN(phi,Gamma,C_aug,Q_multiplier)
%       CONTROLLERDESIGN(phi,Gamma,C_aug,Q_multiplier,R)
%
% Author: Sajid Mohamed

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Default argument values
if nargin < 3
    error('Not enough input arguments for simulation. For details - type >>help controllerDesign');
end
if nargin < 4
    Q_multiplier=1;
end
if nargin < 5
    R=1; 
end
%% Design per scenario
fprintf('LQR Controller Design for each scenario\n')
for i=1:size(phi,2)
    %% Initialise
    n=length(phi{i});
    %% augmentation for the integral tracking action, if using LQI
    %     phi_aug{i} = [phi{i}  zeros(n,1); 
    %                 C_aug{i}        1];
    %     Gamma_aug{i} = [Gamma{i}; 0];
    %     C_augI{i} = [C_aug{i} 0];
    %% check for controllability
    controllability = ctrb(phi{i},Gamma{i});
    det(controllability);
    if det(controllability) == 0
        fprintf('\tScenario s_%d is Uncontrollable and needs decomposition\n',i);
        clear phi_ctr Gamma_ctr C_ctr T k phi_controlled Gamma_controlled C_controlled K_controlled K_T;
        [phi_ctr,Gamma_ctr,C_ctr,T,k] = ctrbf(phi{i},Gamma{i},C_aug{i});
        no_uncontrollable_states = length(phi_ctr)-sum(k);
        phi_controlled = phi_ctr(no_uncontrollable_states+1:n, no_uncontrollable_states+1:n);
        Gamma_controlled = Gamma_ctr(no_uncontrollable_states+1:n);
        C_controlled = C_ctr(no_uncontrollable_states+1:n);
    %         C_controlled = C_aug(no_uncontrollable_states+1:n+gamma+1);
    %         Q = [zeros(n+gamma-no_uncontrollable_states, n+gamma-no_uncontrollable_states) zeros(n+gamma-no_uncontrollable_states,1);zeros(1,n+gamma-no_uncontrollable_states)  2];
        Q = Q_multiplier*eye(n - no_uncontrollable_states);
        %% Design using dare
        [~,~,G] = dare(phi_controlled, Gamma_controlled, Q,R);
        K_controlled = -G;
        F{i} = 1/(C_controlled*inv(eye(n - no_uncontrollable_states)-(phi_controlled+Gamma_controlled*K_controlled))*Gamma_controlled);
        K_T = K_controlled;   
        %% Augmenting uncontrollable states
        for j=1: no_uncontrollable_states
            K_T = [0 K_T];
        end 
        %% Controller Gain
        K{i} = K_T*T;
    else
        fprintf('\tScenario s_%d is Controllable\n',i);
        Q = Q_multiplier*[zeros(n, n) zeros(n,1);zeros(1,n)  10^10];
        [~,~,G] = dare(phi{i}, Gamma{i}, Q, R);
        K{i} = -G;
    end
end
cqlf_Ai{i}= phi{i} + (Gamma{i}*K{i});
fprintf('===========================================================\n');
