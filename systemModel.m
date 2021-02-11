function [A,B,C,D] = systemModel(MODEL)
% SYSTEMMODEL - to define the LKAS system model (dynamics) 
%   Arguments:
%       MODEL: to choose the state-space matrices of the model
%   Returns:
%       A, B, C, D : state-space matrices. Currently, this function returns matrices in continuous-time domain.
%   Usage:
%       SYSTEMMODEL()
%       SYSTEMMODEL(MODEL)

% 	Reference: This LKAS system model is described in 
%				1. J. Kosecka, et al., "Vision-based lateral control of vehicles", in Proceedings of Conference on Intelligent Transportation Systems, IEEE, 1997, pp. 900–905.
%				2. S. Mohamed, et al., "Designing image-based control systems considering workload variations", in: 58th IEEE Conference on Decision and Control (CDC), 2019.
%
%   Author: Sajid Mohamed (s.mohamed@tue.nl)
%   Organization: Eindhoven University of Technology
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% LKAS system model described in Mohamed et al, DSD 2018, CDC 2019, MICPRO 2020
% Author: Sajid Mohamed
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Default argument values
if nargin < 1
    MODEL = 3; % 1: VREP, 2: WEBOTS, 3: LKAS from paper, 4: Suspension Control System (DEFAULT) 
end
if MODEL==1
    vrepLKAS;
elseif MODEL==2
    webotsLKAS;
elseif MODEL==3
    mohamedPaperLKAS;
elseif MODEL==4
    suspensionSystem;
else
    error("Undefined SYSTEM_MODEL %d in config.\n Check systemModel.m for possible options.",MODEL);
end