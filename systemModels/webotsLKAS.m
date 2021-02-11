%% Define the LKAS system model for Webots
%   Author: Sajid Mohamed (s.mohamed@tue.nl)
%   Organization: Eindhoven University of Technology
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Parameters
velocityCar = 50; %in kmph
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