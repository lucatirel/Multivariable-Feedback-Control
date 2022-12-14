%% MFC, Lanari, Second Homework, Motor
% Student: Tirel Luca 1702631
% Created on: 28/03/21
% Last Update: 03/04/21

%This File is used for the Plots (run after Simulink)

%% POINT 1
% Plot L1,S1,W11,T1,W31
figure(1)
sigma(L,'g',S,'b',W1,'b--',T,'r',W3,'r--')
legend('L','S','W1','T','W3','Location','southeast')
title('Sensitivity Analysis for Speed Control Problem')
grid on;

% Plot L1,W11,1/W31
figure(2)
sigma(L,'g',W1,'b--',1/W3,'r--')
legend('L','W1','1/W3','Location','southeast')
title('Cascade Loop Dynamics and Performance Bounds')
grid on;

% Plot W1,W2,W3
figure(3)
sigma(W1,'b',W2,'g',W3,'r')
legend('W1','W2','W3','Location','southeast')
title('Weighting Functions')
grid on;
% 
% % Control Input (from Simulink)
% figure(4)
% plot(out.T,out.u_MS)
% legend('Control Input u','Location','southeast')
% title('Voltage Control Input')
% xlabel('Time (s)') 
% ylabel('Voltage (V)') 
% grid on
% 
% % Step response comparison with Mixsyn and Hinfsyn
% figure(5)
% step(sys,sys1,opt)
% legend('Mixsyn Controller','Hinfsyn Controller','Location','southeast')
% title('Step Response with Mixsyn and Hinfsyn')
% grid on;
% 

%% POINT 2 and 3

% Comparison between L and Ln of the Cascade Loop Step Responses with same Controller Gn
figure(6)
step(sys,'b',sysn,'r',opt)
legend('Nominal Plant','Plant with Neglected Dynamics','Location','southeast')
title('Step Responses Comparison with the same Controller')
grid on;

% Comparison of Cascade Loop Functions
figure(7)
bode(L,'b',Ln,'r')
legend('Cascade Loop with Inductance','Cascade Loop without Inductance','Location','southwest')
title('Comparison of the Bode Diagrams of the Loop Functions')
grid on;

% Comparison of Poles and Zeroes
figure(8)
subplot(2,1,1)
pzmap(sys)
title('Zeroes and Poles of Cascade Loop Nominal Plant')
subplot(2,1,2)
pzmap(sysn)
title('Zeroes and Poles of Cascade Loop Plant with Neglected Dynamics')

% Comparison of Nyquist Plots
figure(9)
nyquist(L,'b',Ln,'r')
legend('Cascade Loop with Inductance','Cascade Loop without Inductance','Location','southeast')
title('Nyquist Plot of Cascade Loop Functions')

% Comparison of Nominal Process and Process with a Weighted Neglected Dynamic  
figure(10)
bode(Pn_w,'b',P,'r',Pn,'g',om)
legend('Process with Weight for the Neglected Dynamic','Nominal Process','Process without Inductance','Location','northeast')
title('Bode Plot of the Process with a Weight on the Neglected Dynamic')
grid on;

% High pass filter to weight neglected dynamic
figure(11)
bode(Wn1,om)
legend('Weight that represent Neglected Dynamic','Location','northeast')
title('Bode Plot of the Weight')
grid on;

%% POINT 4

% Plots of L,S,T with Uncertainties 
figure(12)
subplot(3,1,1)
bodemag(loop_u.Li,'g')
legend('L_u','Location','southeast')
title('Loop Function with Uncertainties')
grid on;
subplot(3,1,2)
bodemag(loop_u.Si,'b',W1,'b--')
legend('S_u','W1','Location','southeast')
title('Sensitivity Function with Uncertainties')
grid on;
subplot(3,1,3)
bodemag(loop_u.Ti,'r',W3,'r--')
legend('T_u','W3','Location','southeast')
title('Complementary Sensitivity Function with Uncertainties')
grid on;

% Nyquist Plot of L_u
figure(13)
nyquist(loop_u.Li)
legend('L_u','Location','southeast')
title('Nyquist Plots of Cascade Loop with Uncertainties')

% Bode Plot of Closed Loop System with Uncertainties and Nominal Plant G
figure(14)
bode(sysu)
legend('Closed Loop System with Uncertainties','Location','southeast')
title('Bode Plot of Closed Loop System with Uncertainties')
grid on;

% Step and Impulse Response of Closed Loop System
figure(15)
step(sysu,opt);
legend('Step Response','Location','southeast')
title('Step Response of the Closed Loop Dynamics with Uncertainties in the Plant')
xlabel('Time (s)') 
ylabel('Speed (rpm)') 
grid on;