clear
close all
clc

%% Quadcopter parameters from the report in correct units

m   = 0.5;        % kg
L   = 0.25;       % m
k   = 3e-6;       % N s^2
b   = 1e-7;       % N m s^2
g   = 9.81;       % m/s^2
kd  = 0.25;       % kg/s
Ixx = 5e-3;       % kg m^2
Iyy = 5e-3;       % kg m^2
Izz = 1e-2;       % kg m^2
cm  = 1e4;        % V^(-2) s^(-2)

Ts = 0.05;        % sampling time

%% Equilibrium point derivation

% State vector (given):
% x = [x y z vx vy vz phi theta psi omega_x omega_y omega_z]'

x_eq = zeros(12,1);

% Input vector (given):
% u = [v1^2 v2^2 v3^2 v4^2]'
% u = [u1 u2 u3 u4]'
%
% At hover we have:
% m*g = k*cm*(u1 + u2 + u3 + u4)
% and u1 = u2 = u3 = u4.

u_hover = m*g/(4*k*cm);
u_eq = u_hover*ones(4,1);

% Output vector (given):
% y = [x y z phi theta psi]'

y_eq = [x_eq(1);
        x_eq(2);
        x_eq(3);
        x_eq(7);
        x_eq(8);
        x_eq(9)];

disp('Hover equilibrium input u_eq = ')
disp(u_eq)

%% Continuous-time linearized model around hover point
%
% Deviation variables/model:
% dx_dot = A*dx + B*du
% dy     = C*dx

A = zeros(12,12);
B = zeros(12,4);

% Position equations (see report)
A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;

% Translational velocity equations (see report)
A(4,4) = -kd/m;
A(5,5) = -kd/m;
A(6,6) = -kd/m;

% Coupling between attitude and horizontal acceleration (see report)
A(4,8) = g;
A(5,7) = -g;

% Vertical acceleration input (see report)
B(6,:) = (k*cm/m)*[1 1 1 1];

% Attitude kinematics (see report)
A(7,10) = 1;
A(8,11) = 1;
A(9,12) = 1;

% Angular acceleration inputs (see report)
B(10,:) = (L*k*cm/Ixx)*[1 0 -1 0];
B(11,:) = (L*k*cm/Iyy)*[0 1 0 -1];
B(12,:) = (b*cm/Izz)*[1 -1 1 -1];

% Output matrix (see report)
C = zeros(6,12);
C(1,1) = 1;   % x
C(2,2) = 1;   % y
C(3,3) = 1;   % z
C(4,7) = 1;   % phi
C(5,8) = 1;   % theta
C(6,9) = 1;   % psi

D = zeros(6,4); (see report)

disp('Continuous-time A matrix:')
disp(A)

disp('Continuous-time B matrix:')
disp(B)

%% Continuous-time state-space system

sysc = ss(A,B,C,D);  % Continuous

disp('Continuous-time poles:')
disp(eig(A))

%% Discretization with zero-order hold

sysd = c2d(sysc, Ts, 'zoh');  % Discretize using ZOH method (see report)

Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;
Dd = sysd.D;

disp('Discrete-time poles:')
disp(eig(Ad))

%% Basic system analysis for in report

n_states = size(Ad,1);

rank_ctrb = rank(ctrb(Ad,Bd));
rank_obsv = rank(obsv(Ad,Cd));

disp('Rank of controllability matrix:')
disp(rank_ctrb)

disp('Rank of observability matrix:')
disp(rank_obsv)

if rank_ctrb == n_states
    disp('The discrete-time model is controllable.')
else
    disp('The discrete-time model is NOT fully controllable.')
end

if rank_obsv == n_states
    disp('The discrete-time model is observable.')
else
    disp('The discrete-time model is NOT fully observable.')
end

disp('Transmission zeros of the discrete-time model:')
disp(tzero(sysd))

% (see report for all conclusions of above code)

%% Save matrices for later scripts (LQR/LQR/Pole placement design)

save('quadcopter_linear_model.mat', ...
     'm','L','k','b','g','kd','Ixx','Iyy','Izz','cm','Ts', ...
     'x_eq','u_eq','y_eq', ...
     'A','B','C','D','Ad','Bd','Cd','Dd','sysc','sysd')