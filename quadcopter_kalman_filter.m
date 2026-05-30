%*************************************************
% Design of a Kalman Filter
% for the quadcopter
%*************************************************

clear all;
clc;

load("quadcopter_linear_model.mat")
load("references_01.mat");
load("Ki.mat")
load("Ks.mat")


%%
%*******************************************
% System matrices of the linearized model
% around:
%*******************************************

% Operating point
x_star = [0 0 0 0 0 0 0 0 0 0 0 0];  %-> body frame aligns with inertial frame
u_star = [40.8750 40.8750 40.8750 40.8750]; %-> > thrust cancels out gravity


% Full state vector: x, y, z, vx, vy, vz, phi, theta, psi, wx, wy, wz
% Measured States (Output): x, y, z, phi, theta, psi
Cd = zeros(6,12);

Cd(1,1) = 1;  % x 
Cd(2,2) = 1;  % y
Cd(3,3) = 1;  % z

Cd(4,7) = 1;  % phi
Cd(5,8) = 1;  % theta
Cd(6,9) = 1;  % psi

Dd = zeros(6, 4);

%  x[n+1] = Ad x[n] + Bd u[n] + B1 w[n]    {State equation}
%  y[n]   = Cd x[n] + Dd u[n] +    v[n]    {Measurements}

% From Assignment
% B1 = I_12



%%
%******************************************************
% Computation of the Kalman Filter Gain via DLQE
%******************************************************

noise_var_pos = 2.5e-5;    % Noise variance for   x,     y,   z -> 2.50 * 10^-5
noise_var_angle = 7.57e-5; % Noise variance for phi, theta, psi -> 7.57 * 10^-5

% Process Noise:
Q = 1e-4*eye(12); % TUNING 

% Measurement Noise:
R = diag([ ...
    2.5e-5 2.5e-5 2.5e-5 ...    % x, y, z
    7.57e-5 7.57e-5 7.57e-5 ... % phi, theta, psi
]);

% Computing L
fprintf('\nThe Kalman filter gain is:\n');
M  = dlqe(Ad, eye(12), Cd, Q, R);
L = Ad * M


fprintf('\nThe estimator poles are:\n');
eig(Ad - Ad * M * Cd)


%%
%******************************************************
% Design of the Compensator with Integral Action
%******************************************************

% State Equations
%
% \hat{x}_{k+1} = [Ad - Bd*Ks - L*Cd, -Bd*Ki]  * [\hat{x}_k; x_I_k] + L*y_k
% x_I_{k+1} = x_I_k - r_k

% Control Law
%
% u_k = [-Ks -Ki] [\hat{x}_k; x_I_k]


Ad_bar = [
    Ad-Bd*Ks-L*Cd, -Bd*Ki;
    zeros(3,12), eye(3);
];

fprintf('\nThe compensator poles are:\n');
eig(Ad_bar)

% Proves the stability of the computed compensator
fprintf('\nThe absolute value of the compensator poles are:\n');
abs(eig(Ad_bar))

% The first 12 values correspond to the system without integral action 
% These are all inside the unit circle

% The last 3 correspond to the integrators 
% These lie on the unit circle (1, 0)