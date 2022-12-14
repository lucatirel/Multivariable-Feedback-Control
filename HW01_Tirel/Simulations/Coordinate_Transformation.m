%% Define System
% System
A = [1 2 3;
     1 3 -2;
     1 3 -2]
 
B = [1; 2; 7]

C = [8 3 7;]

n = length(A)

% Find eigenvectors
[eig_v_dx, Diag, eig_v_sx] = eig(A)


%% Coordinate Tranformations

% Find T^-1
T_inv = eig_v_dx

% Find T
T = inv(T_inv)

% Find New Coordinates for the system
A_new = T*A*T_inv

B_new = T*B

C_new = C*T_inv





