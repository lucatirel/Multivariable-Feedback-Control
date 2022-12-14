%% MFC, Lanari, Fourth Homework
% Student: Tirel Luca 1702631
% Created on: 01/06/21
% Last Update: 05/06/21
clear all
close all
clc

s = tf('s');

%% Parameters (Nominal)
% (source:https://it.mathworks.com/help/control/ug/dc-motor-control.html)
Ra = 2;          % Ohm
La = 0.5;        % Henry 
Jm = 0.02;       % Kg*m^2 
Bm = 0.1;        % N*m/(rad/s)     
km = 0.1;        % N*m/A
ke = 0.1;        % V/(rad/s)
% Disturbances
Tl = 0.1;           % N*m  Load Torque

D = tf(Tl);

% Model Uncertainties on the parameters
Jm1 = ureal('Jm1',Jm,'Percentage',10);
Ra1 = ureal('Ra1',Ra,'Percentage',10);
La1 = ureal('La1',La,'Percentage',10);
Bm1 = ureal('Bm1',Bm,'Percentage',10);
km1 = ureal('km1',km,'Percentage',10);
ke1 = ureal('ke1',ke,'Percentage',10);

% Disturbances Uncertainties
Tl1 = ureal('Tl1',Tl,'Percentage',100);

D1 = tf(Tl1);

% Transfer Functions (without uncertainties)
% Armature
A = tf([km],[La Ra]);

% Motor
M = tf([1],[Jm Bm]);

% Motor Internal Feedback
KE = tf(ke);

%--------------------------------------------------------------------------

%% AUGMENTED PLANT (NOMINAL)
%INPUTS              OUTPUTS
% w(1) = r          z(1) = WS
% w(2) = Tl         z(2) = WU
% w(3) = n          z(3) = WT
% w(4) = u          z(4) = Measurment Error

% Armature
A.u = 'v1';
A.y = 'Tm';

% Disturbances
D.u = 'w(2)';
D.y = 'Tl';

% Torque Input with Disturbances
sum1 = sumblk('T = Tm - Tl');   % T = Tm - Tl

% Rotor
M.u = 'T';
M.y = 'y';

% Internal Feedback
KE.u = 'y';
KE.y = 'v2';

sum2 = sumblk('v1 = w(4) - v2');     % v1 = Vm - v2

sum3 = sumblk('ym = y + w(3)');    % ym = y + n

sum4 = sumblk('z(4) = w(1) - ym'); % em = r - ym

%% CONTROLLER SYNTHESIS
% Define weight for Sensitivity S(s)
W1 = makeweight(10^2, 10^0, 0.01);     %low pass filter to reject disturbances

% Define Weight for Control Sensitivity Su(s)
W2 = makeweight(0.00001, 10^1.5, 10^4);

% Define Weight for Complementary Sensitivity T(s)
W3 = makeweight(0.01, 10^1, 10^2);       % high pass filter to reduce the effect of the noise at HF

% Including Weights in the Interconnected Model
W1.u = 'z(4)';
W1.y = 'z(1)';

W2.u = 'w(4)';
W2.y = 'z(2)';

W3.u = 'y';
W3.y = 'z(3)';

% Creation of the Augmented Plant 
P1 = connect(A,M,KE,D,W1,W2,W3,sum1,sum2,sum3,sum4,'w','z');  % Augmented plant with no control loop
size(P1)

% H-infinity Controller (1 measurement, 1 control input)
[K,CL,gamma] = hinfsyn(P1,1,1);

%% CLOSED LOOP AUGMENTED PLANT DEFINITION (NOMINAL)
%INPUTS              OUTPUTS
% w(1) = r          z(1) = WS
% w(2) = Tl         z(2) = WU
% w(3) = n          z(3) = WT          
%                   z(4) = theta dot
%                   z(5) = control

% Fake Blocks
I = tf(1);
I1 = tf(1);

% Controller Block
K.u = 'em';
K.y = 'Vm';

% Redefining Signals
%(we must consider exogenous control and error output as internal signals)
sum2 = sumblk('v1 = Vm - v2');     % v1 = w(4) - v2
sum4 = sumblk('em = w(1) - ym');   % em = r - ym

W1.u = 'em';
W2.u = 'Vm';

% Adding Thetadot and Control Input as Output (just to see them)
I.u = 'y';
I.y = 'z(4)';

I1.u = 'Vm';
I1.y = 'z(5)';

% Creation of Closed Loop Augmented Plant
P2 = connect(A,M,KE,D,W1,W2,W3,K,I,I1,sum1,sum2,sum3,sum4,'w','z'); % Augmented plant with control loop
size(P2)


%--------------------------------------------------------------------------


%% AUGMENTED PLANT (UNCERTAIN)
% Transfer Functions (with uncertainties)
% Armature
A1 = tf([km1],[La1 Ra1]);

% Motor
M1 = tf([1],[Jm1 Bm1]);

% Motor Internal Feedback
KE1 = tf(ke1);

%% AUGMENTED PLANT (UNCERTAIN)
%INPUTS              OUTPUTS
% w(1) = r          z(1) = WS
% w(2) = Tl         z(2) = WU
% w(3) = n          z(3) = WT
% w(4) = u          z(4) = Measurment Error

% Uncertain Armature
A1.u = 'v1';
A1.y = 'Tm';

% Uncertain Disturbances
D1.u = 'w(2)';
D1.y = 'Tl';

sum1 = sumblk('T = Tm - Tl');   % T = Tm - Tl

% Uncertain Motor
M1.u = 'T';
M1.y = 'y';

% Motor Internal Uncertain Feedback
KE1.u = 'y';
KE1.y = 'v2';

sum2 = sumblk('v1 = w(4) - v2');     % v1 = Vm - v2

sum3 = sumblk('ym = y + w(3)');    % ym = y + n

sum4 = sumblk('z(4) = w(1) - ym'); % em = r - ym

%% CONTROLLER SYNTHESIS
% Define weight for Sensitivity S(s)
W1 = makeweight(10^2, 10^0, 0.01);     %low pass filter to reject disturbances

% Define Weight for Control Sensitivity Su(s)
W2 = makeweight(0.00001, 10^1.5, 10^4);

% Define Weight for Complementary Sensitivity T(s)
W3 = makeweight(0.01, 10^1, 10^2);       % high pass filter to reduce the effect of the noise at HF

% Including Weights in the Interconnected Uncertain Model
W1.u = 'z(4)';
W1.y = 'z(1)';

W2.u = 'w(4)';
W2.y = 'z(2)';

W3.u = 'y';
W3.y = 'z(3)';

% Creation of the Uncertain Augmented Plant 
P1u = connect(A1,M1,KE1,D1,W1,W2,W3,sum1,sum2,sum3,sum4,'w','z');  % Augmented plant with no control loop
size(P1u)

% Mu Synthesis Controller
[K1,gamma1] = musyn(P1u,1,1); 

%% CLOSED LOOP AUGMENTED PLANT WITH NOMINAL CONTROLLER and MUSYN ONE
%INPUTS              OUTPUTS
% w(1) = r          z(1) = WS
% w(2) = Tl         z(2) = WU
% w(3) = n          z(3) = WT          
%                   z(4) = theta dot


% Controller Links (Nominal Hinfsyn Controller)
K.u = 'em';
K.y = 'Vm';

% Controller Links (Nominal Musyn Controller)
K1.u = 'em';
K1.y = 'Vm';

% Definitions of Exogenous Inputs as Internal Ones
sum2 = sumblk('v1 = Vm - v2');     % v1 = Vm - v2
sum4 = sumblk('em = w(1) - ym');   % em = r - ym

% Redefining Signals
%(we must consider exogenous control and error output as internal signals)
W1.u = 'em';
W2.u = 'Vm';

% Define Scope Output
I.u = 'y';
I.y = 'z(4)';


%Hinfsyn Controller Closed Loop System
P2u_K = connect(A1,M1,KE1,D1,W1,W2,W3,K,I,sum1,sum2,sum3,sum4,'w','z'); % Augmented plant with control loop
% Musyn Controller Closed Loop System
P2u_K1 = connect(A1,M1,KE1,D1,W1,W2,W3,K1,I,sum1,sum2,sum3,sum4,'w','z'); % Augmented plant with control loop

size(P2u_K)
size(P2u_K1)
gamma1


%--------------------------------------------------------------------------

%% ROBUST STABILITY AND PERFORMANCE ANALYSIS
% Robust Stability
opts = robOptions('Display','on','Sensitivity','on');
[stabmarg,wcu,info] = robstab(P2u_K1,opts);
stabmarg
wcu

% Robust Performance
opt = robOptions('Display','on');
[perfmarg,wcu1] = robgain(P2u_K1,3,opt);
perfmarg

% Find the Worst Uncertainty Case
[maxgain,worstuncertainty] = wcgain(P2u_K1);   % worst case
P2u_K1_worst = usubs(P2u_K1,worstuncertainty); 
worst_CL_hinfnorm = norm(P2u_K1_worst,inf)                         % hinf norm of CL in worst case

% Extracting the Uncertain Part of the System
[N,DELTA,BLKSTRUCT] = lftdata(P2u_K1);


%% PLOTS
% Control input minimization is taken into account but its magnitude is not
% considered in the following work (could be improved with a frequency
% dependent weight)

% Noise TimeSpace
tn = 0:0.0000001:1;

% Noise Signal
n = 1*sin(1000*tn);

% Reference Signal
opt_ref = stepDataOptions('StepAmplitude',15);  % in rad/s

% Nominal Plant with Nominal Controller
x = tf(P2(4,:));  % Transfer Functions from Inputs to Thetadot
y = tf(P2(5,:));  % Transfer Functions from Inputs to Control

% Theta Dot
figure(1)
subplot(3,1,1)
step(x(1),opt_ref)   % r-y response
subplot(3,1,2)
step(x(2))           % d-y response
subplot(3,1,3)
lsim(x(3),n,tn)      % n-y response

% Control Effort 
figure(2)
subplot(3,1,1)
step(y(1),opt_ref)   % r-y response
subplot(3,1,2)
step(y(2))           % d-y response
subplot(3,1,3)
lsim(y(3),n,tn)      % n-y response

% Uncertain Controlled Plant Output with "Nominal-Designed" vs "Musyn"
figure(3)
step(P2u_K(4,1),'b',P2u_K1(4,1),'r',P2u_K1(4,1).nominal,'g',opt_ref)  % r-y response
title('Output Reponse of the Uncertain System wrt input : r (Hinfsyn vs Musyn)')
legend('Uncertain Plant with Hinfsyn Controller','Uncertain Plant with Musyn Controller','Nominal Plant with Hinfsyn Controller','Location','southeast')

figure(4)
step(P2u_K(4,2),'b',P2u_K1(4,2),'r',P2u_K1(4,2).nominal,'g')          % d-y response
title('Output Reponse of the Uncertain System wrt input : Tl (Hinfsyn vs Musyn)')
legend('Uncertain Plant with Hinfsyn Controller','Uncertain Plant with Musyn Controller','Nominal Plant with Hinfsyn Controller','Location','southeast')

figure(5)
step(P2u_K(4,3),'b',P2u_K1(4,3),'r',P2u_K1(4,3).nominal,'g')
title('Output Reponse of the Uncertain System wrt input : n (Hinfsyn vs Musyn)')
legend('Uncertain Plant with Hinfsyn Controller','Uncertain Plant with Musyn Controller','Nominal Plant with Hinfsyn Controller','Location','northeast')

                                      % n-y response (can not plot sinusoidal 
                                      %               response of uncertain 
                                      %               plant in Matlab,
                                      %               so i assume constant
                                      %               noise disturbance
% Worst Case vs Nominal
figure(6)
step(P2u_K1_worst(4,1),'b', P2u_K1(4,1).nominal,'g', opt_ref)
title('Output Reponse wrt input : r (Nominal vs Worst)')
legend('Worst System Response','Nominal System Response','Location','southeast')


