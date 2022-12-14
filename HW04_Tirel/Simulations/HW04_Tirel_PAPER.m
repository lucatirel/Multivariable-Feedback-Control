clear all
close all
clc

s = tf('s');
t = 0:0.001:20;
om = logspace(-4,4,10000);

% I made a mistake and tried to reproduce the first part of the paper

% %% EXAMPLE 5: DECOUPLER REALIZABILITY (NOT NEEDED FOR HOMEWORK)
% % Undelayed System
% G11 = tf([1],[1 2]);
% G12 = tf([-1],[1 2]);
% G21 = tf([1 -0.5],[1 4 4]);
% G22 = tf([1 -1 0.25],[2 12 24 16]);
% 
% % Checking Realizability
% det = G11*G22 - G21*G12;
% 
% % Delayed System
% G=[G11*exp(-2*s) G12*exp(-6*s); 
%    G21*exp(-3*s) G22*exp(-8*s)];
% 
% % Checking Realizability
% det = G11*G22 - G21*G12;
% % Poles and Zeroes
% pp = pole(G);
% zz = tzero(G) % one positive zero (z=0.5)
%               % moreover extra delay 
%               % no phase margin
% 
% PM = allmargin(G).PhaseMargin;
% 
% % Check Zero with Lower Multiplicity
% m1 = size(zero(G21),1)   % This m = 1 + lower delay of row
% m2 = size(zero(G22),1)  % This m = 2
% 
% % We should choose dd11 != 0 to decouple but 
% % dd21 is in the same column (input channel) !
% % SO WE ADD EXTRA DELAY ALONG u1
% 
% % Adding Extra Time Delay of -4 Unit in The First Input Channel
% % to achieve realizability
% G11n = tf([1],[1 2])*exp(-6*s);
% G21n = tf([1 -0.5],[1 4 4])*exp(-7*s);
% 
% Gn = [G11*exp(-6*s) G(1,2); 
%       G21*exp(-7*s) G(2,2)];
% 
% % Now minimum phase element can be choose in (1,1) as well as in (1,2)
% % and we can choose dd12 in (1,2) and of course dd21 in (2,1)
% 
% 
% %% Decoupling
% Dd = [0 1;
%       1 0];
%   
% do11 = -G11/G12;   %-g11/g12
% do12 = 0;
% do21 = 0;
% do22 = -G22/G21;   %-g22/g21
% 
% % Adding Delay Manually
% DEL1 = 6-6;
% DEL2 = 8-7;
% do11 = do11*exp(-DEL1*s);
% do22 = do22*exp(-DEL2*s);
% 
% Do = minreal([do11 do12;
%               do21 do22]);
% % Decoupling Matrix
% D = Dd*inv((eye(size(Dd))-Do*Dd));
% 
% % Decoupled Process
% PD = pade(D*G,1);
% W1 = makeweight(10^3, .01, 0.01);
% W3 = makeweight(0.01, .015, 10^2);
% 
% % Controller Synthesis (Decoupled vs Not Decoupled)
% Ga = pade(G,1);
% [K,CL,gamma] = mixsyn(PD,W1,[],W3);
% [K1,CL1,gamma1] = mixsyn(Ga,W1,[],W3);
% 
% % Closed Loop System
% CL = feedback(K*PD,eye(size(K*PD)));
% CL1 = feedback(K1*Ga,eye(size(K1*Ga)));

% figure(1)
% step(CL,'g',CL1,'r')

%--------------------------------------------------------------------------

%% Example 4.1
% Undelayed Plant
G11 = tf([22.89],[4.572 1]);
G12 = tf([-11.64],[1.807 1]);
G21 = tf([4.689],[2.174 1]);
G22 = tf([5.80],[1.801 1]);

% Undelayed Plant
G = [G11 G12;
     G21 G22];

% Determinant (with delays)
Det = G11*G22*exp(-0.6*s) - G21*G12*exp(-0.6*s);

% Delayed Plant
Gd = [G11*exp(-0.2*s) G12*exp(-0.4*s);
     G21*exp(-0.2*s) G22*exp(-0.4*s)];

% Highlight Negative Phase Margin      
pp = eig(Det);             % + Stable Plant (from poles point of view) 
zz = tzero(Det);           % + M.P. + Negative Gain Margin (for delays)
GPM = allmargin(Det)       % = Unstable Plant

% From this Delayed Plant we can see that both the "bigger" delays appear
% in the second input (column).
% We need to introduce an extra delay along the first input channel

% Extra Delay Block
N = [exp(-0.2*s) 0;
     0           1];
 
% Decoupler-Realizable Plant
Gn = Gd*N

% Plot of the Response of The System Adding Extra Delay
figure(2)
step(Gd,Gn,t)
title('Step Response of the Open Loop System (Nominal vs Extra Delayed)')
legend('Nominal System','Delayed System','Location','southeast')

% Determinant of the Delayed System
Detn = Gn(1,1)*Gn(2,2) - G(2,1)*G(1,2);

% Decoupling Configuration (1,2)
Ga = pade(Gn,0);

Dd = eye(2);

do11 = 0;
do22 = 0;
do12 = -Ga(1,2)/Ga(1,1);
do21 = -Ga(2,1)/Ga(2,2);

Do = [do11 do12;
      do21 do22];

% Decoupling Matrix
D = inv(Dd-Do);
size(D)

% Pid Controllers
Kp1 = 0.157;
Kp2 = 0.244;
Ti1 = 4.57;
Ti2 = 1.8;
K1 = pidstd(Kp1,Ti1,0);
K2 = pidstd(Kp2,Ti2,0);
K = [K1 0;
     0 K2];

% Decoupled Open Loop System
OL_DEC = Gn*D;

% Decoupled Closed Loop System
CL_DEC = feedback(OL_DEC*K,eye(size(OL_DEC*K)));

% Undecoupled Closed Loop System
CL_NDEC = feedback(Gd*K,eye(size(Gd*K)));

% PID RESPONSE (Decoupled vs Non Decoupled)
figure(3)
step(CL_DEC,'g',CL_NDEC,'r',t);
title('Step Response of the Closed Loop System (Decoupled + Delayed vs Non Decoupled)')
legend('Nominal Delayed and Decoupled System','Non Decoupled System','Location','southeast')

%% Comparison with other controllers of the papers
% Xiong Controller
KK1 = [0.3137*(1+(1/(4.572*s))) 0.2203*(1+1/(2.174*s));
      -0.0369*(1+1/(1.807*s))   0.2439*(1+1/(1.801*s))];
sys1 = feedback(Gd*KK1,eye(size(Gd)));

% Lee
KKKp1 = 0.133;
KKKp2 = 0.19;
TTTi1 = 6.47;
TTTi2 = 2.61;
KKK_1 = pidstd(KKKp1,TTTi1,0);
KKK_2 = pidstd(KKKp2,TTTi2,0);
KK3 = [KKK_1 0;
      0 KKK_2];
sys3 = feedback(Gd*KK3,eye(size(Gd)));

% Comparison
figure(5)
step(CL_DEC,'g',CL_NDEC,'g--',sys1,'b',sys3,'r',t);
title("Closed Loop Systems Comparisons of the Controller Proposed");
legend('Nominal Delayed and Decoupled System','Non Decoupled System','Xiong','Lee','Location','southeast')



%--------------------------------------------------------------------------



%% Creating Augmented Uncertain Plant
% INPUTS      OUTPUTS
% w(1) = d    z(1) = ym
% w(2) = r

% Performance and Uncertainty Weights
WI = ((0.1*s + 0.2)/(0.05*s + 1))*eye(2);
WP = (((s/2.2 + 0.3)/s))*eye(2);

% Define Uncertainty Block
DI = ultidyn('Delta',[2 2]);

% Controller
K.u = 'v';
K.y = 'u';

% Weight Fo Uncertainty
WI.u = 'u';
WI.y = 'x';

% Uncertainty Block
DI.u = 'x';
DI.y = 'ud';

% Diagonal Mulitplicative input uncertainty entry
sum1 = sumblk('ur = u + ud',2);

% Decoupled Delayed Plant
OL_DEC.u = 'ur';
OL_DEC.y = 'y';

% Unecoupled Nominal Plant
Gd.u = 'ur';
Gd.y = 'y';

% Disturbance Entry (assumed constant)
sum2 = sumblk('ym = y + d',2);

% Performance Weight
WP.u = 'ym';
WP.y = 'z';

% Feedback Sum
sum3 = sumblk('v = r - ym',2);

% Creating Plant
% no output disturbances are considered but included in the model

% Decoupled + Delayed Closed Loop Plant
Pu_CL = connect(K,WI,DI,OL_DEC,WP,sum1,sum2,sum3,{'r'},{'ym'});
% Undecoupled Closed Loop Plant
Pu_NDEC_CL = connect(K,WI,DI,Gd,WP,sum1,sum2,sum3,{'r'},{'ym'});

% Step Response of the Uncertain Closed Loop System
figure(4)
step(Pu_CL,'g', Pu_NDEC_CL,'r', Pu_CL.Nominal,'b', t)
title('Step Response of the Uncertain Closed Loop System (Decoupled + Delayed vs Non Decoupled vs Nominal)')
legend('Delayed and Decoupled Uncertain CL System','Non Decoupled Uncertain CL System','Nominal Delayed and Decoupled CL System','Location','southeast')

% Robust Stability and Performances
opt = robOptions('Display','on','Sensitivity','on','MussvOptions','a');

% Decoupled and Delayed
[stabmarg,info11] = robstab(Pu_CL,opt);
[perfmarg,info12] = robgain(Pu_CL,1.5,opt);
stabmarg
perfmarg

% Non Decoupled
[stabmarg1,info21] = robstab(Pu_NDEC_CL,opt);
[perfmarg1,info22] = robgain(Pu_NDEC_CL,1.5,opt);
stabmarg1
perfmarg1