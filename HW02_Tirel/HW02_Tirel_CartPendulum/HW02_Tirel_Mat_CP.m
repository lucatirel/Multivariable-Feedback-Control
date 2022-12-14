%% MFC, Lanari, Second Homework, Cart and Inverted Pendulum Dynamics
% Student: Tirel Luca 1702631
% Created on: 04/04/21
% Last Update: 04/04/21

clear all
clc

t = 0:0.00001:10; 
om= logspace(-10,14,1000000);

s = tf('s');

% Angle Offset and reference input
x0 = -(15*pi)/180;
opt1 = stepDataOptions('InputOffset',x0,'StepAmplitude',-x0);

% Position rference input
opt2 = stepDataOptions('StepAmplitude',5);

%% Cart Pendulum Model
% Model Parameters
M = 0.5;                % Mass of the cart [Kg]
m = 0.2;                % Mass of the pendulum [Kg] 
l = 0.3;                % Distance between pendulum center of mass and pivot [m]
J = 0.006;              % Inertia of the pendulum around the center of mass [Kg*m^2]

% Friction
%c1 = 0.1;               % Viscous Friction coefficient [N/m/sec]
%c2 = 20;                % Coulomb Friction torque [N*m]
c1 = 0;
c2 = 0;

% Other Parameters
g = 9.8;                % Gravity constant [m/s^2]

Mt = m + M;             % Total Mass [Kg]
Jt = J + m*l^2;          % Pendulum Angular Momentum around pivot [Kg*m^2]
Dd = Mt*J + M*m*(l^2);

%--------------------------------------------------------------------------
%% State Space Model
% Definition of the S.I.M.O. state space model
% Dynamics
A = [0 0 1 0;                                     % cart position wrt origin [p]
     0 0 0 1;                                     % angle between the vertical axis and the pendulum [theta]
     0 (-(m^2)*(l^2)*g)/Dd -(c1*Jt)/Dd (c2*l*m)/Dd;  % cart speed [p_dot]
     0 (Mt*m*g*l)/Dd (c1*l*m)/Dd (-c2*Mt)/Dd];       % angular speed of the pendulum [theta_dot]

% Input Matrix
B = [0;
     0;
     Jt/Dd;
     (-m*l)/Dd];

% Output Matrix
C = [0 1 0 0;           % output pendulum angle [theta]
     1 0 0 0;];           % output cart position [p]
 
% Disturbances Matrix
D = [0;
      0];

% State Space Model Definition
sys_t = ss(A,B,C,D);

% Controllability and Observability check
rank(ctrb(A,B));
rank(obsv(A,C));

% Transfer Functions of the System
H = tf(sys_t);

H_u_theta = H(1);                    % Extract tf from input u to output theta
H_u_p = H(2);                        % Extract tf from input u to output p
H_p_theta = H_u_theta*(H_u_p^-1);    % Extract tf from input p to output theta

P1 = zpk(H_u_theta)
P2 = zpk(H_u_p)
P3 = zpk(H_p_theta)


%--------------------------------------------------------------------------

%% Theta Control
% (we have one rhp pole)
% Poles
pp = pole(P1);

% Weight for Sensitivity
W11 = makeweight(10^2,1,0.01);

% Weight for Complementary Sensitivity
wbT1 = 15;        %this must be bigger then 11.1682 = 2 * 5.5841 = 2*p
MT1 = 2;
AT1  = 0.01;                   
W31 = ((s+wbT1/MT1)/(AT1*s+wbT1));

% Optimal Controller Synthesis
[K1,CL1,gamma1] = mixsyn(P1,W11,0,W31);

% Definition of Cascade Open Loop System
L1 = zpk(K1*P1);
I = eye(size(L1));
S1 = feedback(I,L1); 
T1 = I-S1;

% System Info
allmargin(L1);  %check that PMFrequency is bigger then 11.1682 rad/s

% Feedback Controlled System Input/Angle
sys1 = zpk(feedback(K1*P1,1));

%% Position Control 
% Weight for Sensitivity S(s)
wbS2 = 0.1;          
AS2 = 0.01;          % DC Gain (inverse) 
MS2 = 2;              % HF Gain 
W12 = (((s/MS2)+wbS2)/(s+wbS2*AS2));              

% Weight for Control
W22 = tf(0.001);  

% Weight Parameters for Complementary Sensitivity
wbT2 = 15;          
MT2 = 2;             
AT2  = 0.1;          
W32 = ((s+wbT2/MT2)/(AT2*s+wbT2));  

% Controller
[K2,CL2,gamma2] = mixsyn(P2,W12,W22,W32);

% Definition of Cascade Open Loop System
L2 = zpk(K2*P2);
I = eye(size(L2));
S2 = feedback(I,L2); 
T2 = I-S2;

% System Info
allmargin(L2);

% Feedback Controlled System Input/Position
sys2 = zpk(feedback(K2*P2,1));  % NOT STABLE !!!!
                                % theta will diverge!!!!
sys_int = zpk(feedback(K2*P1,1));

