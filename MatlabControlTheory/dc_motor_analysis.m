% DC Motor Speed: System Analysis

% System Model

J = 0.01;                               %moment of inertia
b = 0.1;                                %damping
K = 0.01;                               %motor constant
R = 1;                                  %armature resistance
L = 0.5;                                %inductance
s = tf('s');
P_motor = K/((J*s+b)*(L*s+R)+K^2);

%step response
linearSystemAnalyzer('step', P_motor, 0:0.1:5);
 
%first order simpkification poles
rP_motor = 0.1/(0.5*s+1)
linearSystemAnalyzer('step', rP_motor, 0:0.1:5);

%proportional control
Kp = 100;
C = pid(Kp);
sys_cl = feedback(C*P_motor,1);
t = 0:0.01:5;
step(sys_cl,t)
grid
title('Step Response with Proportional Control')
