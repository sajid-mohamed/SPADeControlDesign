function [K,F,cqlf_Ai] = controlDesign(phi,Gamma,C_aug,Q,R,CONTROLLER_TYPE)
% CONTROLDESIGN - choose the correct control design based on
% CONTROLLER_TYPE

% Author: Sajid Mohamed
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Default argument values
if nargin < 6
    CONTROLLER_TYPE = 1;  
end
if CONTROLLER_TYPE == 1 %LQR
    [K,F,cqlf_Ai] = controllerDesignLQR(phi,Gamma,C_aug,Q,R);
elseif CONTROLLER_TYPE == 2 %LQI
    [K,F,cqlf_Ai] = controllerDesignLQI(phi,Gamma,C_aug,Q,R);
else
    error("Undefined CONTROLLER_TYPE %d in config.\n Check controlDesign.m for possible options.",CONTROLLER_TYPE);
end