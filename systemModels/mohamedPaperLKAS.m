%% Define the LKAS system model (dynamics) 
% 	Reference: This LKAS system model is described in 
%				1. J. Kosecka, et al., "Vision-based lateral control of vehicles", in Proceedings of Conference on Intelligent Transportation Systems, IEEE, 1997, pp. 900–905.
%				2. S. Mohamed, et al., "Designing image-based control systems considering workload variations", in: 58th IEEE Conference on Decision and Control (CDC), 2019.
%   LKAS system model used in Mohamed et al, DSD 2018, IMACS 2019, CDC 2019, 
%                                            MICPRO 2020, CDC 2020 
%   Author: Sajid Mohamed (s.mohamed@tue.nl)
%   Organization: Eindhoven University of Technology
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
velocityCar = 50; %in kmph
vx = 15; %longitudinal velocity(m/s)
LL = 15;  %look ahead distance(m)

m = 1590; %mass of car(kg)
I_psi = 2920; %interia of CG(kg*m^2)
l_f = 1.22; %distance from CG to front axle(m)
l_r = 1.62; %distance from CG to rear axle (m)
c_f = 2 * 60000; %(N/rad)
c_r = 2 * 60000; %cornering stiffness is increased by factor 2 -> 2 tires lumped together (N/rad)
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