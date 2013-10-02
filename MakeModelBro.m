clc; clear all; close all;

syms x y z xd yd zd theta phi thetad phid T1 T2 T3 T4 m g Ix Iy L1 L2 m g;
X = [x y z xd yd zd theta phi thetad phid];
f = [X(4) X(5) X(6) T1/m*sin(-theta)+T3/m*sin(-theta) T2/m*sin(phi)+T4/m*sin(phi)...
    T1/m*cos(-theta)+T3/m*cos(-theta)+T2/m*sin(phi)+T4/m*sin(phi)-m*g X(9) X(10)...
    1/Ix*(T4-T2)*L2 1/Iy*(T3-T1)*L1];
% No [g,m] term? Might need to add to state?

A =  jacobian(f,X);

U = [T1 T2 T3 T4];

B = jacobian(f, U);

C = eye(length(X));

D = zeros(length(X),length(U));

%% Make some values for the real system
g   = 9.81; %m/s
L1  = 0.2;  %m
L2  = L1;
m   = 2;    %kg 
m_mot = m/4 - (0.75*m)/4;
Ix  = m_mot*L1;
Iy  = m_mot*L2;

%% Pick an equilibrium reference trajectory
T1 = m*g/4; T2 = T1; T3 = T1; T4 = T1;
theta = 0; phi = theta;

A = double(subs(A));
B = double(subs(B));

QuadCopterModelJizz = ss(A,B,C,D);



%% See if system is controllable
ControlAbility = ctrb( QuadCopterModelJizz ); 

%% See if system is observalbe
Observability = obsv(QuadCopterModelJizz);
% if rank of controlAbility and OBseravability - #states, then we' are
% touch in butthole like a boss!

