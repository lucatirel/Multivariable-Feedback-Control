%% PLOTS
% (RE-RUN AFTER HAVING RUN SIMULINK)

% Poles and Zeroes (SPEED CONTROL)
figure(1)
pzmap(G)
title('Zero Poles Map of Open Loop System for Speed Control Problem')
grid on

% Bode OL (SPEED CONTROL)
figure(2)
bode(G)
title('Bode Plots of Open Loop System for Speed Control Problem')
grid on


% Plot L,S,W1,T,W3 (SPEED CONTROL)
figure(3)
sigma(L,'g',S,'b',W1,'b--',T,'r',W3,'r--')
legend('L','S','W1','T','W3','Location','southeast')
title('Sensitivity Analysis for Speed Control Problem')
grid on

% Plot L,W1,1/W3 (SPEED CONTROL)
figure(4)
sigma(L,'g',W1/gamma,'b--',gamma/W3,'r--')
legend('L','W1','1/W3','Location','southeast')
title('Cascade Loop and Weight Functions for Speed Control Problem')
grid on

% Poles and Zeroes (POSITION CONTROL)
figure(5)
pzmap(G1)
title('Zero Poles Map of Open Loop System for Position Control Problem')
grid on

% Bode OL (POSITION CONTROL)
figure(6)
bode(G1)
title('Bode Plots of Open Loop System for Position Control Problem')
grid on

% Plot L1,S1,W11,T1,W31 (POSITION CONTROL)
figure(7)
sigma(L1,'g',S1,'b',W11,'b--',T1,'r',W31,'r--')
legend('L1','S1','W11','T1','W31','Location','southeast')
title('Sensitivity Analysis for Position Control Problem')
grid on;

% Plot L1,W11,1/W31 (POSITION CONTROL)
figure(8)
sigma(L1,'g',W11,'b--',1/W31,'r--')
legend('L1','W11','1/W31','Location','southeast')
title('Cascade Loop and Weight Functions for Position Control Problem')
grid on;

%--------------------------------------------------------------------------

% Thetadot Comparison
figure(9)
plot(out.T,out.Thetadot_OL)
legend('Thetadot','Location','southeast')
title('Angular Speed - OpenLoop Speed Control')
xlabel('Time (s)') 
ylabel('Angular Speed (rpm)') 
grid on

%--------------------------------------------------------------------------

% SPEED CONTROL PROBLEM
% Thetadot Comparison
figure(10)
plot(out.T,out.Thetadot_PID1,out.T,out.Thetadot_MS1)
legend('Thetadot-PID','Thetadot-MS','Location','southeast')
title('Angular Speed - Speed Control')
xlabel('Time (s)') 
ylabel('Angular Speed (rpm)') 
grid on

% Control Input Comparison
figure(11)
plot(out.T,out.u_PID1,out.T,out.u_MS1)
legend('Control-PID','Control-MS','Location','southeast')
title('Control Input - Speed Control')
xlabel('Time (s)') 
ylabel('Voltage (V)') 
grid on

% Current Comparison
figure(12)
plot(out.T,out.I_PID1,out.T,out.I_MS1)
legend('Current-PID','Current-MS','Location','southeast')
title('Current - Speed Control')
xlabel('Time (s)') 
ylabel('Current (A)') 
grid on

%--------------------------------------------------------------------------

% POSITION CONTROL PROBLEM
% Theta Comparison
figure(13)
plot(out.T,out.Theta_PID2,out.T,out.Theta_MS2)
legend('Theta-PID','Theta-MS','Location','southeast')
title('Angular Position - Position Control')
xlabel('Time (s)') 
ylabel('Angular Position (r)') 
grid on

% Thetadot Comparison
figure(14)
plot(out.T,out.Thetadot_PID2,out.T,out.Thetadot_MS2)
legend('Thetadot-PID','Thetadot-MS','Location','southeast')
title('Angular Speed - Position Control')
xlabel('Time (s)') 
ylabel('Angular Speed (rpm)') 
grid on

% Control Input Comparison
figure(15)
plot(out.T,out.u_PID2,out.T,out.u_MS2)
legend('Control-PID','Control-MS','Location','southeast')
title('Control Input - Position Control')
xlabel('Time (s)') 
ylabel('Voltage (V)') 
grid on

% Current Comparison
figure(16)
plot(out.T,out.I_PID2,out.T,out.I_MS2)
legend('Current-PID','Current-MS','Location','southeast')
title('Current - Position Control')
xlabel('Time (s)') 
ylabel('Current (A)') 
grid on

% Comparison of weights applications in Nyquist plots
figure(17)
nyquist(G*Ka,'r',G*Kb,'g',G*K,'b')
title('Comparison of the Cascade Loop Functions when applying weights sequentially')

% since my phase margin of L = G*K is about 90deg, the roll off condition
% tells me that the slope at crossover f. of L is equal to -2+2*((pi/2)/pi) = -1
% and since it is less then -2 i respected the constraint on the steepness
% of L around its crossover frequency.



