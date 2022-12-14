%% MFC, Lanari, Third Homework, Distillation Process
% Student: Tirel Luca 1702631
% Created on: 10/05/21
% Last Update: 19/05/21


clear all
clc

% Variable Definition
s = tf('s');

% Gravity Constant
g = 9.81; % m/s^2

% Nominal Levels of height of waters in the Tanks
h1o = 16.3; %cm 
h2o = 13.7; %cm 
h3o = 6.0; %cm 
h4o = 8.1; %cm 

% Ratio of flow 1-4 
gam1 = 0.3;
% Ratio of flow 2-3
gam2 = 0.3;

% Tank Area
A1 = 730; % cm^2
A2 = 730; % cm^2
A3 = 730; % cm^2
A4 = 730; % cm^2

% Drain Area
a1 = 2.05; % cm^2
a2 = 2.26; % cm^2
a3 = 2.37; % cm^2
a4 = 2.07; % cm^2

% Pumps Proportionallity Constants
k1 = 7.45; % cm^3/s %
k2 = 7.30; % cm^3/s %

% Constant in Output Model
kt = 1;

% Time Constants in the Linear System
tau1 = (A1/a1)*sqrt(2*h1o/g);
tau2 = (A2/a2)*sqrt(2*h2o/g);
tau3 = (A3/a3)*sqrt(2*h3o/g);
tau4 = (A4/a4)*sqrt(2*h4o/g);

n1 = kt*tau1/A1;
n2 = kt*tau2/A2;
n3 = kt*tau3/A3;
n4 = kt*tau4/A4;

% Reference Steps
opt1 = stepDataOptions('StepAmplitude',h1o);
opt2 = stepDataOptions('StepAmplitude',h2o);



% State Space Model
% Dynamics
A = [-1/tau1 0 A3/(A1*tau3) 0;
     0 -1/tau2 0 A4/(A2*tau4);
     0 0 -1/tau3 0;
     0 0 0 -1/tau4];
% Inputs
B = [(gam1*k1)/A1 0;
     0 (gam2*k2)/A2;
     0 ((1-gam2)*k2)/A3;
     (1-gam1)*k1/A4 0];
% Outputs
C = [kt 0 0 0;
     0 kt 0 0];
% Disturbances
D = 0;

% Dimensions
n = size(A,1);
p = size(B,2);
q = size(C,1);

% State Space and Transfer Functions Models
SS = ss(A,B,C,D);
W = tf(SS)

% Poles and Transmission Zeroes
ppp = pole(W);
zzz = tzero(W)
RHP_zero = max(zzz)

% Static Gain
DC = dcgain(W)
condition_n = cond(DC)


%% Creation of the Augmented Plant and Decoupler + PID
% Plant I/O
W.InputName = {'u1','u2'};
W.OutputName = {'y'};

% Decoupler I/O
Dec = tunableGain('Decoupler', eye(p));
Dec.InputName = 'e';
Dec.OutputName = {'p_u1','p_u2'};

% Tunable PID controllers and I/O
PI_u1 = tunablePID('PI_u1','pi');
PI_u1.InputName = 'p_u1';
PI_u1.OutputName = 'u1';

PI_u2 = tunablePID('PI_u2','pi');
PI_u2.InputName = 'p_u2';
PI_u2.OutputName = 'u2';

% Links creation
sum1 = sumblk('e = r - y',2);

% Augmented Plant without Controller
C0 = connect(PI_u1,PI_u2,Dec,sum1,{'r','y'},{'u1','u2'});

% Desired Loop Bandwidth
wc = 0.0005;

% Obtaining PID Controllers + Decoupler and Infoes
[W,K1,gamma1,Info] = looptune(W,C0,wc);

% Display the PID Controllers Parameters
showTunable(K1)

% Define the Closed Loop System with Decoupler + PI in parallel
T = connect(W,K1,'r','y');

% Display PID Controller Performances and Infoes
Perf_K1 = stepinfo(T)
Perf_PID1 = Perf_K1(1);
Perf_PID2 = Perf_K1(2);

%% Mixed Sensitivity H-inf Synthesis
% Definitions of Weights
W1 = makeweight(10^5, 0.002, 0.01); % Low-Pass Filter
W3 = makeweight(10^-2, 1, 10^2);    % High-Pass Filter

% Controller Synthesis (trying to achieve gam=1)
[K2,CL2,gamma2] = mixsyn(W,W1,0,W3,1);

% Definition of Closed Loop Synthesis with Loop Shaping Controller
T1 = feedback(W*K2,1);

% Display Loop Shaping Controller Performances and Infoes
Perf_K2 = stepinfo(T1)
Perf_1 = Perf_K2(1);
Perf_2 = Perf_K2(2);

% Plots of the response are shown in Simulink and in the Doc File. Mixsyn
% obtained better results. The tuning method used (by me) for PI
% controllers did not specified performance objectives and required a
% really slow response with a crossover frequency between (0.0001,1), while
% i required more stringent specifications for the Loop Shaping controller
% The two controllers could be set equivalent by imposing the same time
% constant. The RHP zero affect the output imposing limitation on the
% bandwidth.


%% Unstructured Uncertainty Case - Robust Stability
% Definitions of Uncertain Parameters for the Input Links
gam1u = ureal('gam1u',gam1,'Percentage',10);
gam2u = ureal('gam2u',gam2,'Percentage',10);
k1u = ureal('k1u',k1,'Percentage',10);
k2u = ureal('k2u',k2,'Percentage',10);

% Uncertain State Space Model
Bu = [(gam1u*k1u)/A1 0;
     0 (gam2u*k2u)/A2;
     0 ((1-gam2u)*k2u)/A3;
     (1-gam1u)*k1u/A4 0];
SSu = ss(A,Bu,C,D);

% Uncertain Transfer Functions Matrix
Wu = [(gam1u*k1u*n1)/(1+tau1*s) ((1-gam2u)*k2u*n1)/((1+tau1*s)*(1+tau3*s));
       ((1-gam1u)*k1u*n2)/((1+tau2*s)*(1+tau4*s)) (gam2u*k2u*n2)/(1+tau2*s)];

% Definitions of Weights for Multiplicative Input Uncertainty

% (These weights are taken from given reference 3 and these
% are consistent since we use the same parameters. A nice way
% to visualize this is in the plot number 4. We should have that
% the magnitude of the weight chosen is bigger than the smaller singular
%value of inv(W)*(Wu-W), and require that this weight has an hinfnorm
%less then one

WI1=(s+0.4*0.12)/((s/0.22)+0.12);
WI2=(s+0.4*0.12)/((s/0.22)+0.12);

% Hinfnorm check
hinfnorm(WI1);

% Definitions of Delta Blocks and Block Diagonal Matrix for Uncertainties
Delta1 = ultidyn('Delta1',[1 1]);
Delta2 = ultidyn('Delta2',[1 1]);

% Definitions of the Delta Upper Block for Multiplicative Uncertainties and Hinfnorm check 
Delta = blkdiag(1+WI1*Delta1,1+WI2*Delta2);
hinfnorm(Delta)

% To check that the controller we have found before
% stabilize the Uncertain Plant Robustly, we could have a look
% to the Nyquist Plot of the cascade of perturbed plant with
% the controller. If there are no encirclements of the point (-1,0)
% we can conclude that for all the perturbations in the set we have defined
% we still have stability (Robust Stability condition is satisfied)

% Definitions of Perturbed Loop Function
Lp = Wu*K2;

%% Robust Performance (not yet completed)
% Definition of Perturbed System
Wp = W*Delta;

W1p = makeweight(10^5, 0.002, 0.01);
W3p = makeweight(10^-2, 1, 10^2); 

[K3,CL3,gamma3] = mixsyn(W,W1,0,W3,1);

LS = loopsens(Wp,K3);










% Linear Fractional Transformation (unused)
% Paug = augw(W,W1,0,W3);
% N = lft(Paug,K2);
% F = lft(N,Delta);



%% Theory
%% In order to obtain robust stability we need: (by Skogestad pag 300)
% 1) F = Fu(N,Delta) stable
% 2) N is Stable
% 3) hinfnorm(Delta) less or equal then one

%% In order to obtain robust performance we need: (by Skogestad pag 300)
% 1) hinfnorm(F) strictly less then one
% 2) hinfnorm(Delta) less then one
% 3) nominal stability





%% Plots
% Display Loop Infoes and Stability Margins for Decoupler + PI
figure(1)
loopview(W,K1,Info);

% Step of CL certain and uncertain system
figure(2)
subplot(1,2,1)
step(feedback(W*K2,eye(size(W*K2))),opt1,opt2)
subplot(1,2,2)
step(feedback(Wu*K2,eye(size(W*K2))),opt1,opt2)

%Singular Values of Certain and Uncertain Systems
figure(3)
subplot(2,1,1)
sigma(W)
subplot(2,1,2)
sigma(Wu)

% Weight for Input Multiplicative Uncertainty
figure(4)
sigma(WI1,'g',inv(W)*(Wu-W))
legend('WI', 'sigma(inv(W)*(Wu-W))')

% Check of Robust Stability Condition
figure(5)
nyquist(Lp)

