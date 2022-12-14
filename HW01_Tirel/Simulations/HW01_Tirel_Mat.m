%% MFC, Lanari, First Homework
% Student: Tirel Luca 1702631
% Created on: 16/03/21
% Last Update: 30/05/21

clc
s = tf('s');

%% Parameters (Nominal)
% Motor Parameters
Ra = 8.8;            % Ohm
La = 28*10^-6;       % Henry 
Jm = 9.5*10^-10;     % Kg*m^2 
Bm = 1.02*10^-9;     % N*m/rpm
km = 0.00109;        % N*m/A
ke = 0.000114;        % V/rpm
% Disturbances
Ts = 0.000011;       % N*m  Static friction torque
ST = 0.000732;       % N*m
Tl = ST/2;           % N*m  Load torque

% Nominal Input
Vm = 6;              % V    Nominal Voltage

%--------------------------------------------------------------------------

%% SPEED CONTROL PROBLEM (with PID in Simulink)
% Transfer Functions
% Armature
A = tf([km],[La Ra]);

% Motor
M = tf([1],[Jm Bm]);

% Open Loop Function without Disturbances
G = feedback(A*M,ke)

% Including Load and Stiction Torques Disturbances (not used)
X = feedback(A*M-M*(Tl+Ts),ke)

%% Open Loop Analysis
% Stability
[Gm,Pm,Wcg,Wcp] = margin(G)

% Open Loop Bandwidth
fb_OL = bandwidth(G)

%% Mixed Sensitivity Loop Shaping Design

% My controller was synthetized by considering the free-disturbance plant
% We could have included knowledge of disturbances inside the nominal plant

% Define weight for Sensitivity S(s)
W1 = makeweight(10^2.5, 50, 0.001);     %low pass filter to reject disturbances

% Define Weight for Control Sensitivity Su(s)
W2 = makeweight(0.1, 1000, 10^2);       % high pass filter to reduce control effort at HF

% Define Weight for Complementary Sensitivity T(s)
W3 = makeweight(0.001, 50, 10^2);       % high pass filter to reduce the effect of the noise at HF

% Controller Synthesis
% weights are chosen for the third case and perform better in that
% situation. We could change closed loop specifications in the first two
% cases (Ka and Kb) by choosing less conservative weights.

[Ka,CLa,gammaa] = mixsyn(G,W1,0,0);     
[Kb,CLb,gammab] = mixsyn(G,W1,W2,0);    
[K,CL,gamma] = mixsyn(G,W1,W2,W3);      

% Definition of Controlled System
L = G*K;
I = eye(size(L));
S = feedback(I,L); 
T = I-S;

% Analysis of Loop Function
allmargin(L)

%--------------------------------------------------------------------------

%% POSITION CONTROL PROBLEM (with PID in Simulink)
% Define Integrator for Position
integ = tf(1/s);
 
%% SPEED CONTROL PROBLEM (with PID in Simulink)
% Define Open Loop Transfer Function + Integrator in Series
G1 = G*integ

%% Open Loop Analysis
% Stability
[Gm1,Pm1,Wcg1,Wcp1] = margin(G1)

% Define weight for Sensitivity S(s)
W11 = makeweight(10^3,10^1.8,0.1)^2;

% Define Weight for Control Sensitivity Su(s)
W21 = tf(5);

% Define Weight for Complementary Sensitivity T(s)
W31 = makeweight(0.1,10^3,10^3);

% Controller Synthesis
[K1,CL1,gamma1] = mixsyn(G1,W11,W21,W31,1);

% Loop Analysis
L1 = G1*K1;
I = eye(size(L1));
S1 = feedback(I,L1);
T1 = I-S1;

% Analysis of Loop Function
allmargin(L1);

%--------------------------------------------------------------------------