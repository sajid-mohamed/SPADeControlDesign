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
    MODEL = 3; % 1: VREP, 2: WEBOTS, 3: from paper (DEFAULT) 
end
%% Parameters
velocityCar = 50; %in kmph
%% MODEL parameters
if MODEL==1 %VREP
    %% model parameter setting (VREP)
    % The dimensions of the VREP model are scaled down 1:10. 
    vx = 2.2; %longitudinal velocity(m/s)
    LL = 0.55;  %look ahead distance(m) [less than 1s*Vx]
    
    m = 1.599374; %mass of car(kg)
    I_psi = 0.0705323934; %interia of CG(kg*m^2) [I=m*r^2]
    l_f = 0.21; %distance from CG to front axle(m)
    l_r = 0.21; %distance from CG to rear axle (m)
    c_f = 2 * 60; %(N/rad)
    c_r = 2 * 60; %cornering stiffness is increased by factor 2 -> 2 tires lumped together (N/rad)
elseif MODEL==2 %WEBOTS
    %% model parameter setting (WEBOTS)
    vx = (velocityCar)*5/18 %longitudinal velocity(m/s) (20 km/hr = 5.55556)
    LL = 5.5;  %look ahead distance(m) [less than 1s*Vx]

    m = 2000; %mass of car(kg)
    I_psi = 6337.74; %interia of CG(kg*m^2) [I=m*r^2]
    %I_psi = 63.3774; %interia of CG(kg*m^2) [I=m*r^2]
    l_f = 1.6975; %distance from CG to front axle(m)
    l_r = 1.2975; %distance from CG to rear axle (m)
    c_f = 2 * 60000; %(N/rad)
    c_r = 2 * 60000; %cornering stiffness is increased by factor 2 -> 2 tires lumped together (N/rad)
else %PAPER
    vx = 15; %longitudinal velocity(m/s)
    LL = 15;  %look ahead distance(m)
    
    m = 1590; %mass of car(kg)
    I_psi = 2920; %interia of CG(kg*m^2)
    l_f = 1.22; %distance from CG to front axle(m)
    l_r = 1.62; %distance from CG to rear axle (m)
    c_f = 2 * 60000; %(N/rad)
    c_r = 2 * 60000; %cornering stiffness is increased by factor 2 -> 2 tires lumped together (N/rad)
end
%% Model Variables
a1 = c_f + c_r;
a2 = c_r * l_r - c_f * l_f;
a3 = -l_f * c_f + l_r * c_r;
a4 = l_f^2 * c_f + l_r^2 * c_r;
b1 = c_f / m;
b2 = l_f * c_f / I_psi;

%% state parameter description
% x1 is Vy
% x2 is yaw rate(rad/s)
% x3 is yL ==> F(S)=yL(S)/KL(S)
% x4 is epsilon_L

A = [-a1/(m*vx)     (a2-m*vx^2)/(m*vx)   0   0   0;
     a3/(I_psi*vx)  -a4/(I_psi*vx)       0   0   0;
     -1             -LL                  0   vx  0;
     0              -1                   0   0   vx;
     0              0                    0   0   0];
 
B = [b1; b2; 0; 0; 0];
C= [0 0 1 0 0];
D=0;