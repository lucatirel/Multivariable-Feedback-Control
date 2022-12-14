%% MFC, Lanari, Second Homework, Motor
% Student: Tirel Luca 1702631
% Created on: 28/03/21
% Last Update: 03/04/21

clear all
clc

s = tf('s');
om = logspace(-2,10,10000);

% Definition of Reference Exogenous Input
opt = stepDataOptions('StepAmplitude',25000);

%% Motor Parameters (Nominal Values)
% Motor Parameters
Ra = 8.8;            % Ohm
La = 28*10^-6;       % Henry 
Jm = 9.5*10^-10;     % Kg*m^2 
Bm = 1.02*10^-9;     % N*m/rpm
km = 0.00109;        % N*m/A
ke = 0.000114;        % V/rpm 

% Disturbances
Ts = 0.000011;       % N*m  Static friction torque
MTl = 0.000732;       % N*m  Maximum Load Torque Allowed
Tl = MTl/2;           % N*m  Load torque

% Nominal Input
Vm = 6;              % V    Nominal Voltage

%--------------------------------------------------------------------------
%% SPEED CONTROL PROBLEM
%% Motor Dynamics
% Armature
A = tf([km],[La Ra]);

% Rotor
M = tf([1],[Jm Bm]);

% Open Loop Function
P = feedback(A*M,ke);

% Nominal System Analysis
[Gm,Pm,Wcg,Wcp] = margin(P);
bandwidth(P);

%% Mixed Sensitivity Loop Shaping Design
% Define weight for Sensitivity S(s)
W1 = makeweight(10^2.5, 50, 0.001); 

% Define Weight for Control Sensitivity Su(s)
W2 = makeweight(0.1, 1000, 10^2);

% Define Weight for Complementary Sensitivity T(s)
W3 = makeweight(0.001, 50, 10^2);

% Augmented Plant Based Controller Synthesis
[K,CL,gamma] = mixsyn(P,W1,W2,W3);

% Definition of Controlled System
L = P*K;
I = eye(size(L));
S = feedback(I,L); 
T = I-S;

% System Analysis
allmargin(L)

% Closed Loop System Definition (with Mixsyn controller)
sys = feedback(L,1);

%% H-infinity Design Method
% Augmented Plant Definition with Same Weights
Pe = augw(P,W1,W2,W3);

% Optimal Controller Synthesis
[K1,CL1,gamma1] = hinfsyn(Pe,1,1);

% Definition of Cascade Open Loop System
L1 = P*K1;
I = eye(size(L1));
S1 = feedback(I,L1); 
T1 = I-S1;

% Closed Loop System Definition (with Hinfsyn controller)
sys1 = feedback(L1,1);

%--------------------------------------------------------------------------
%% Neglection of High Frequencies Dynamics
% Armature with Neglected Inductance
An = tf([km],[Ra]);

% Open Loop System
Pn = feedback(An*M,ke);

% Creation of the Gn with Mixed Synthesis Design Method (same weights are used)
[Kn,CLn,gamman] = mixsyn(Pn,W1,W2,W3);

% Definition of Cascade Open Loop System
Ln = Pn*Kn;
I = eye(size(Ln));
Sn = feedback(I,Ln); 
Tn = I-Sn;

% Analysis of Cascade Loop Function
allmargin(Ln)

% Closed Loop System Definition
sysn = feedback(Ln,1);

% Changing Closed Loop Specifications (Better results in u and y)
W1n = makeweight(10^3, 100, 0.01);
W2n = makeweight(0.1, 1000, 10^2);
W3n = makeweight(0.001, 110, 10^2);
[Kn1,CLn1,gamman1] = mixsyn(Pn,W1n,W2n,W3n);

% Definition of Cascade Open Loop System (new specifications)
Ln1 = Kn1*Pn;
I = eye(size(Ln1));
Sn1 = feedback(I,Ln1); 
Tn1 = I-Sn1;

%% Find a Weight to Represent the Neglected Dynamic
% Weight for Plant that mimic the Neglection of Inductance
tau_n = La/Ra;           
tau = ureal('tau',tau_n/2,'Range',[0,tau_n]);
Wn = 1/(1+tau*s);

% Weighted Plant
Pn_w = Pn*Wn;

% Weight for the Neglected Dynamic (high pass filter)
Wn1 = tau*s/(1+tau*s)

%--------------------------------------------------------------------------
%% Study of the Uncertain Model
% Model Uncertainties on the parameters
Jm1 = ureal('Jm1',Jm,'Percentage',20);
Ra1 = ureal('Ra1',Ra,'Percentage',20);
La1 = ureal('La1',La,'Percentage',20);
Bm1 = ureal('Bm1',Bm,'Percentage',40);
km1 = ureal('km1',km,'Percentage',40);
ke1 = ureal('ke1',ke,'Percentage',40);

% Disturbances Uncertainties (not used)
Tl1 = ureal('Tl1',Tl,'Percentage',100);
Ts1 = ureal('Ts1',Ts,'Percentage',10000);

% Motor Dynamics (with Uncertainties)
% Stator 
Au = tf([km1],[La1 Ra1]);

% Rotor
Mu = tf([1],[Jm1 Bm1]);

% Open Loop Function
Pu = feedback(Au*Mu,ke1);

% Sensitivity Analysis with Controller Designed for Nominal Plant 
% (no disturbances and noise are considered in nominal plant)
loop_u = loopsens(Pu,K);

% Closed Loop Uncertain System
sysu = feedback(loop_u.Li,1);

% Maximal Parameters Variations Before Instability
[stabmarg,wcu] = robstab(sysu);

%Since our robust stability margins are around 2.5 we could have variations
% 150% bigger then the variations we defined for the parameters, without losing stability.
% Morover the maximal variations of the parameters for which we still have stability 
% are shown by calling wcu
stabmarg
wcu



