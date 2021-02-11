function [phi, Gamma, C_aug] = augmentSystem(tau,h,SYSTEM_MODEL)
% AUGMENTSYSTEM - A function to augment the systemModel() for discrete-time implementation.
%   Arguments: 
%       tau,h: arrays of tau and h values for the scenarios
%   Returns:
%       phi, Gamma, C_aug : augmented state-space matrices in discrete-time
%                           domain. 
%       SYSTEM_MODEL: which system model to choose in systemModel.m
%   Usage:
%       AUGMENTSYSTEM(tau,h)
%       AUGMENTSYSTEM(tau,h,SYSTEM_MODEL)
%   Assumptions: 1) length(tau) = length(h)
%   Dependencies: systemModel.m --> to load the state-space matrices
%
%   Author: Sajid Mohamed

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if nargin < 2
    error('Not enough input arguments. For details - type >>help augmentSystem');
end
if nargin < 3
    SYSTEM_MODEL=3;
end
%% Load system model
[A,B,C,D] = systemModel(SYSTEM_MODEL);
%% Augment System
for i=1:length(tau)
    fprintf('\t Scenario s_%d: tau(%d)=%.3f, h(%d)=%.3f\n',i,i,tau(i),i,h(i));
    sys_continuous = ss(A,B,C,D,'InputDelay',tau(i));
    sys_discrete = c2d(sys_continuous,h(i));
    sysd_aug=absorbDelay(sys_discrete); %% absorbDelay is the key function to augment    
    phi{i}=sysd_aug.a;
    Gamma{i}=sysd_aug.b;
    C_aug{i}=sysd_aug.c;
    clear sys_continuous sys_discrete sysd_aug
end
fprintf('=============================================\n');