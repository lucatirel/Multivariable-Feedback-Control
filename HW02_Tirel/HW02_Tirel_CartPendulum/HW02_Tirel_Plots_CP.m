%% General Plots

% Pole Zero Maps of the I/O system
figure(1)           
subplot(2,1,1)      % 1 pole RHP
pzmap(H_u_theta)    % from u to theta
subplot(2,1,2)      % 1 pole in RHP + 1 zero in RHP + 1 pole/zero in Origin
pzmap(H_u_p)        % from u to p

%--------------------------------------------------------------------------
%% Angle Control

% Plot L1,S1,W11,T1,W31
figure(2)
bodemag(L1,'g',S1,'b',gamma1/W11,'b--',T1,'r',gamma1/W31,'r--',om)
legend('L','S','gamma1/W1','T','gamma1/W3','Location','southwest')
title('Sensitivity Analysis')
grid on;

% Plot L2,W11,1/W31
figure(3)
bodemag(L1,'g',W11/gamma1,'b--',gamma1/W31,'r--',om)
legend('L','W1/gamma1','gamma1/W3','Location','southeast')
title('Cascade Loop Dynamics and Performance Bounds')
grid on;

% Step Response of the Closed Loop System
figure(4)
step(sys1,opt1);
legend('Theta','Location','southeast')
title('Step Response of the Closed Loop system')
xlabel('Time (s)') 
ylabel('Angle (rad)') 

%% Position Control
% Step Response of the Closed Loop System (unstable)
% Position
figure(5)
step(sys2,opt2);
legend('Position of the Cart')
title('Step Response of the Closed Loop system')
xlabel('Time (s)') 
ylabel('Position (m)') 

% Internal Behaviour (angle)
figure(6)
step(sys_int,opt2);
legend('Theta')
title('Step Response of the Closed Loop system')
xlabel('Time (s)') 
ylabel('Angle (rad)') 



