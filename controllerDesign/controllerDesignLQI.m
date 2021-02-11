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
%% LQI Design
fprintf('LQI Controller Design for each scenario\n')
%% augmentation per scenario for the integral tracking action
for i=1:size(phi,2) %iterate over scenarios
    %% Initialise
    n=length(phi{i});
    phi{i} = [phi{i}  zeros(n,1); 
                C_aug{i}        1];
    Gamma{i} = [Gamma{i}; 0];
    C_aug{i} = [C_aug{i} 0];
end
%% Controller gain design using LQR
[K,F,cqlf_Ai] = controllerDesignLQR(phi,Gamma,C_aug,Q,R);
fprintf('===========================================================\n');
