%*************************************************
% Design of a state-feedback tracking controller
% for the quadcopter
%*************************************************

clear all;
clc;

load("references_01.mat");

%%
%*******************************************
% System matrices of the linearized model
% around:
%*******************************************

% Operating point
x_star = [0 0 0 0 0 0 0 0 0 0 0 0];  %-> body frame aligns with inertial frame
u_star = [40.8750 40.8750 40.8750 40.8750]; %-> > thrust cancels out gravity

load("quadcopter_linear_model.mat");

% Assume the entire state vector is available 
Cd = eye(12); Dd = zeros(12, 4);

disp('Discretized Linear model of the plant:');
model = ss(Ad, Bd, Cd, Dd, Ts); 

%************************
%Checking the stability
%************************
fprintf('\n\nPoles:\n');
disp(eig (Ad))

%****************************************************
% Checking if the system is controlable or not
%****************************************************
CO = ctrb(Ad,Bd);
disp('Rank of the controllability matrix:');
rank(CO)

%%
%******************************************************
% Computation of the feedback gain via LQR
%******************************************************

% Setting Qd and Rd

Qd = diag([
    50  50  100 ...   % x y z
    5   5   10  ...   % vx vy vz
    200 200 20  ...   % phi theta psi
    5   5   5         % wx wy wz
]);

Rd = 0.1*eye(4);

%Computing K

fprintf('\nThe state feedback gain is:\n');
K = dlqr(Ad,Bd,Qd,Rd)

%%
%*********************************************
% Reference Input - full state feedback
% Computation of the matrices Nx and Nu
%*********************************************
    
%number of states     
nx = 12;
%number of inputs
nu = 4;
%number of outputs: Assumption: entire state vector is available
ny = 12;

big_A = [Ad-eye(12), Bd;
         eye(nx), zeros(nx, nu)];

big_Y =[ zeros(nx,3)
         eye(nx,3) ];

big_N = big_A\big_Y; 

fprintf('\nReference-Input  full state feedback. The matrices Nx and Nu are:');

Nx = big_N(1:nx,:)
Nu = big_N (nx+1:end,:)

%return

%%
%*******************************************
%  Integral control
%  We add three integrators: tracking x, y, z
%********************************************

Cd = [eye(3) zeros(3, nx-3)];
Dd = zeros(3, 4);

%Constructing the Augmented system
NA = [ eye(3),  Cd;
       zeros(nx,3),  Ad]; % Size 15 x15 

NB = [ Dd 
       Bd]; 
  

%checking the controlabillity of the Augmented system
disp('Rank of the controllability matrix of the augmented system:');
rank(ctrb(NA,NB))


    % Tuning History on Integrators

        % 1) 10  10  50  ...   % Integrators

% Qd and Rd for the augmented system
Qd = diag([
    1  1  1  ...   % Integrators
    50  50  100 ...   % x y z
    5   5   10  ...   % vx vy vz
    200 200 20  ...   % phi theta psi
    5   5   5         % wx wy wz
]);

Rd = 0.1*eye(4);


%computing the feedback matrix of the augmented system
full_K = dlqr(NA,NB, Qd, Rd);

fprintf('\nThe state feedback gains of the augmented system are:\n');

Ki = full_K(:,1:3)
Ks = full_K(:,4:end)


