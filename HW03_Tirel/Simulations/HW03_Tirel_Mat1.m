%% MFC, Lanari, Third Homework, First Part
% Student: Tirel Luca 1702631
% Created on: 10/05/21
% Last Update: 19/05/21

s = tf("s");
G = 10/((s+2)*(s+10))

t = linspace(0,10,1000);
ww = logspace(10^-5,10^5);

%% SISO System
%% Loop Shaping
% Define weight for Sensitivity S(s)
W1 = makeweight(10^6, 40, 0.01);   %This to reject disturbances

% Define Weight for Control Sensitivity Su(s)
W2 = makeweight(0.01, 2000, 10000);

% Define Weight for Complementary Sensitivity T(s)
W3 = makeweight(0.001, 42, 10^5);  %This to reject noise

% Augmented Plant
[C,CL,gam] = mixsyn(G,W1,0,W3,0.999);

% Definition of Controlled SISO System
L = G*C;
I = eye(size(L));
S = feedback(I,L); 
T = feedback(L,1); % (alternative) T=I-S with I = eye(size(L))

% Checking Specifications
wbT = bandwidth(T);
[Gm,Pm,Wcg,Wcp] = margin(L);
SPEC = stepinfo(T);

% MIMO Transfer Functions
Si = 1/(1+C*G);
So = 1/(1+G*C);
Ti = (C*G)/(1+C*G);
To = (G*C)/(1+G*C);

%% Construction of the MIMO System
% Transfer function matrix
%       r     d1   d2   n
W = tf([To   G*So So   -To;            %y
        So  -G*So -So  -So;            %e
        C*So -Ti -C*So -C*So]);        %m

 % Exogenous Variables and Input Vector
d1 = sin(5*t);
d2 = sin(3*t);
n = sin(400*t);
r = ones(10,length(n));

% Static Gain
W_DC = dcgain(W) 

% Sinusoidal Response (not used)
frspd = evalfr(W,5);
frspn = evalfr(W,500);

% Singular Value Decomposition
[u,s,v] = svd(W_DC); % use svd(-,'econ') to reduce size

% Condition Number
cn = max(diag(s))/min(diag(s)); 

% Largest Amplification direction (High Gain)
I_max = v(:,1);   % Input
O_max = u(:,1);  % Output

% Smallest Amplification direction (Low Gain)
I_min = v(:,3);  % Input
O_min = u(:,3); % Output

% Zero Output - Input Direction (at S.S.)
I_null = v(:,4);



% % Controller Synthesis for MIMO (not used)
% W21 = makeweight(10^2, 0.1, 10^-1);
% W23 = makeweight(0.001, 10, 10);

% [K,CL1,gam1] = mixsyn(W, W21, 0, W23);



figure(1)
bodemag(L,S,T)
figure(2)
bodemag([tf(1/W1),tf(1/W3)])
figure(3)
step(T)
figure(4)
sigma(tf(W))


