%% Defines the LKAS system model for VREP 
%   Author: Sajid Mohamed (s.mohamed@tue.nl)
%   Organization: Eindhoven University of Technology
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Parameters
velocityCar = 50; %in kmph
% The dimensions of the VREP model are scaled down 1:10. 
vx = 2.2; %longitudinal velocity(m/s)
LL = 0.55;  %look ahead distance(m) [less than 1s*Vx]

m = 1.599374; %mass of car(kg)
I_psi = 0.0705323934; %interia of CG(kg*m^2) [I=m*r^2]
l_f = 0.21; %distance from CG to front axle(m)
l_r = 0.21; %distance from CG to rear axle (m)
c_f = 2 * 60; %(N/rad)
c_r = 2 * 60; %cornering stiffness is increased by factor 2 -> 2 tires lumped together (N/rad)
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