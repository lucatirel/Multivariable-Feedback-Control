%% Cart Pendulum Model
clc
clear all
close all

s = tf('s');
om = logspace(-5,5);
t = 0:0.01:5;
nt = size(t);

% Model Parameters
M = 0.5;                % Mass of the cart [Kg]
m = 0.2;                % Mass of the pendulum [Kg] 
l = 0.3;                % Distance between pendulum center of mass and pivot [m]
J = 0.006;              % Inertia of the pendulum around the center of mass [Kg*m^2]

% Friction (Neglected)
c1 = 0;
c2 = 0;

% Other Parameters
g = 9.8;                % Gravity constant [m/s^2]

Mt = m + M;             % Total Mass [Kg]
Jt = J + m*l^2;          % Pendulum Angular Momentum around pivot [Kg*m^2]
Dd = Mt*J + M*m*(l^2);

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
     1 0 0 0];         % output cart position [p]

% Disturbances Matrix
D = [0;
     0];

% State Space Model Definition
sys_t = ss(A,B,C,D);

% Controllability and Observability Check
rank(ctrb(A,B));
rank(obsv(A,C));

% Transfer Function
H = tf(sys_t);

% Extract Transfer Functions
H_u_theta = zpk(H(1))                    % Extract tf from input u to output theta
H_u_p = zpk(H(2))                        % Extract tf from input u to output p
H_p_theta = zpk(H_u_theta*(H_u_p^-1))    % Extract tf from input p to output theta
H_theta_p = zpk(inv(H_p_theta))          % Extract tf from input theta to output p

% Define Transfer Function of the System
G = [H_u_p; H_u_theta];
G1 = G(2);
G3 = minreal(H_theta_p);

%% PLANT
% Open Loop Plant (with 2 Output)
% INPUTS            OUTPUTS
% w(1) = u        z(1) = Position
%                 z(2) = Theta

G1.u = 'w(1)';
G1.y = 'z(2)';

G3.u = 'z(2)';
G3.y = 'z(1)';

P = connect(G1,G3,'w','z');
size(P)

%% Poles And Zeroes
pp = pole(P)  % (unstable) -- (p=5.5841) !!!
zz = tzero(P) % (MP)       --  

%% Closed Loop System
% INPUTS             OUTPUTS
% w(1) = r_p         z(1) = Position
% w(2) = r_theta     z(2) = Theta
%                    z(3) = u

I = tf(1);

G1.u = 'u';
G1.y = 'z(2)';

G3.u = 'z(2)';
G3.y = 'z(1)';

% Define the Error Signals
sum1 = sumblk('ep = w(1) - z(1)');
sum2 = sumblk('et = w(2) - z(2)');

% Weight for Sensitivity S(s)
wbS = 10;            % Desired Crossover Frequency for Sensitivity
AS = 0.001;          % DC Gain (inverse) 
MS = 2;              % HF Gain 
W1 = (((s/MS)+wbS)/(s+wbS*AS))*eye(2);              

% Weight for Control
W2 = tf(0.1);  

% Weight Parameters for Complementary Sensitivity
wbT = 30;          % Desired Crossover Frequency for C. Sensitivity
MT = 2;          % DC Gain (inverse)
AT  = 0.001;       % HF Gain     
W3 = ((s+wbT/MT)/(AT*s+wbT))*eye(2);

% Augmented Plant and Controller Synthesis
Pe = augw(P,W1,W2,W3);
[K,CL,gamma]= hinfsyn(Pe,2,1);

% Stabilizing Controller
C = zpk(tf(K))
STK = loopsens(P,K);
L = P*K;


% Include Controller in the Model
K.u = {'ep', 'et'};
K.y = 'u';

% Include Control as Output
I.u = 'u';
I.y = 'z(3)';

% Define Closed Loop Plant
P_CL = connect(G1,G3,I,K,sum1,sum2,'w','z');


%% Comparison of Performances with Case in Which Theta was not Available
P_old = G(1);

zz_old = zero(P_old);     % NMP
pp_old = pole(P_old);     % Unstable

[K1,CL1,gamma1] = mixsyn(P_old,W1(1,1),W2,W3(1,1)); %With the Same Performance NO CONTROLLER FOUND

% Weight for Sensitivity S(s)
wbS1 = 0.1;           % Desired Crossover Frequency for Sensitivity
AS1 = 0.01;          % DC Gain (inverse) 
MS1 = 2;              % HF Gain 
W11 = (((s/MS1)+wbS1)/(s+wbS1*AS1));              

% Weight for Control
W21 = tf(1);  

% Weight Parameters for Complementary Sensitivity
wbT1 = 15;          % Desired Crossover Frequency for C. Sensitivity
MT1 = 2;             
AT1  = 0.1;          
W31 = ((s+wbT1/MT1)/(AT1*s+wbT1));  

% New Stabilizing Controller
[K2,CL2,gamma2] = mixsyn(P_old,W11,W21,W31); %With the Same Performance no Controller
STK1 = loopsens(P_old,K2);
L_old = K2*P_old;
CL2 = feedback(L_old,1);







%% PLOTS
% Creating Step Exogenous Inputs
%   rp = 3 mt   rt = 0 rad
u = [3*ones(nt)' zeros(nt)'];

%Stability Analysis
figure(1)
nyquist(tf(P*K))

% Step Response of the System
figure(2)
lsim(P_CL,u,t)
title('Inverted Cart Pendulum Step Response - Position Angle Control')
xlabel('Time (s)') 
grid on

% Comparison Of Performances SIMO vs SISO
figure(3)
subplot(2,1,1)
sigma(STK.So,'b', STK1.So,'r')
legend('S(s) SIMO','S(s) SISO','Location','southeast')
title('Sensitivity SIMO vs SISO')
grid on

subplot(2,1,2)
sigma(STK.To,'b', STK1.To,'r')
legend('T(s) SIMO','T(s) SISO','Location','southeast')
title('Complementary Sensitivity SIMO vs SISO')
grid on



