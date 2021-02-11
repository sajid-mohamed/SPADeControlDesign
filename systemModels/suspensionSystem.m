%% Suspension Control System State-space
% taken from ctms benchmark

m1 = 2500;
m2 = 320;
k1 = 80000;
k2 = 500000;
b1 = 350;
b2 = 15020;

A=[0                 1   0                                              0
  -(b1*b2)/(m1*m2)   0   ((b1/m1)*((b1/m1)+(b1/m2)+(b2/m2)))-(k1/m1)   -(b1/m1)
   b2/m2             0  -((b1/m1)+(b1/m2)+(b2/m2))                      1
   k2/m2             0  -((k1/m1)+(k1/m2)+(k2/m2))                      0];
B=[0;
   1/m1;
   0;
   (1/m1)+(1/m2)];
%    B=[  0;
%         (b1*b2)/(m1*m2)
%         -(b2/m2)
%         -(k2/m2)];
C=[0 0 1 0];
D=0;