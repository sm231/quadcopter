%*************************************************
% Design of a state-feedback tracking controller
% for the quadcopter
%*************************************************

clear all;
clc;

load("references_01.mat");

%%

hb = find_system(gcs,'Type','Block');
handles = cell2mat(get_param(hb,'Handle'));
arrayfun(@(h) set_param(h,'ShowName','on'), handles);
arrayfun(@(h) set_param(h,'ShowName','on','HideAutomaticName','off'), handles);

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
    50  50  100  ...   % x y z
    10  10  10  ...   % vx vy vz
    200 200 20  ...   % phi theta psi
    10  10  10         % wx wy wz
]);


Rd = 0.05*eye(4);


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
         eye(ny), zeros(ny, nu)];

big_Y =[ zeros(nx,3)
         eye(12, 3) ];

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
    1  1  1   ...   % Extra three states
    50  50  100  ...   % x y z
    10  10  10  ...   % vx vy vz
    200 200 20  ...   % phi theta psi
    10  10  10         % wx wy wz
]);


Rd = 0.005*eye(4);


%computing the feedback matrix of the augmented system
full_K = dlqr(NA,NB, Qd, Rd);

fprintf('\nThe state feedback gains of the augmented system are:\n');

Ki = full_K(:,1:3)
Ks = full_K(:,4:end)


%%
%saveas(figure(1), 'report/figures/lqr/sim_int_top.png')
%saveas(figure(2), 'report/figures/lqr/sim_int_traj.png')
saveas(figure(3), 'report/figures/lqr/sim_int_pos.png')
saveas(figure(4), 'report/figures/lqr/sim_int_angle.png')
saveas(figure(5), 'report/figures/lqr/sim_int_volt.png')


%% Tuning reports

%{ 



%%%%%%%%%%%%%%%%%% FULL-STATE FEEDBACK TUNING %%%%%%%%%%%%%%%%%%%%%

Qd = diag([
    10  10  10  ...   % x y z
    10  10  10  ...   % vx vy vz
    10  10  10  ...   % phi theta psi
    10  10  10         % wx wy wz
]);


Rd = 1*eye(4);

000000000000000000000000000000000000000000000000000000000000000
---------------------------------------------
   Quadcopter exercise - Simulation Report
---------------------------------------------

Full state vector: available
Number of checkpoints reached: 4/7
Checkpoint(s) not reached: 3, 4, 5
Payload: 0 kg

Timing:
-------
From initial pos. to checkpoint 1: 6.500 s
From checkpoint 1 to checkpoint 2: 6.100 s
From checkpoint 2 to checkpoint 3:  ---
From checkpoint 3 to checkpoint 4:  ---
From checkpoint 4 to checkpoint 5:  ---
From checkpoint 5 to checkpoint 6: 4.750 s
From checkpoint 6 to checkpoint 7: 5.650 s

Average time: 5.750 s

---------------------------------------------



Qd = diag([
    50  50  50  ...   % x y z
    10  10  10  ...   % vx vy vz
    10  10  10  ...   % phi theta psi
    10  10  10         % wx wy wz
]);


Rd = 1*eye(4);


11111111111111111111111111111111111111111111111111111111111111
---------------------------------------------
   Quadcopter exercise - Simulation Report
---------------------------------------------

Full state vector: available
Number of checkpoints reached: 7/7
Payload: 0 kg

Timing:
-------
From initial pos. to checkpoint 1: 3.750 s
From checkpoint 1 to checkpoint 2: 5.000 s
From checkpoint 2 to checkpoint 3: 5.550 s
From checkpoint 3 to checkpoint 4: 5.600 s
From checkpoint 4 to checkpoint 5: 5.600 s
From checkpoint 5 to checkpoint 6: 4.400 s
From checkpoint 6 to checkpoint 7: 3.350 s

Average time: 4.750 s

---------------------------------------------




Qd = diag([
    50  50  100  ...   % x y z
    10  10  10  ...   % vx vy vz
    10  10  10  ...   % phi theta psi
    10  10  10         % wx wy wz
]);


Rd = 1*eye(4);


22222222222222222222222222222222222222222222222222222
---------------------------------------------
   Quadcopter exercise - Simulation Report
---------------------------------------------

Full state vector: available
Number of checkpoints reached: 7/7
Payload: 0 kg

Timing:
-------
From initial pos. to checkpoint 1: 3.000 s
From checkpoint 1 to checkpoint 2: 4.200 s
From checkpoint 2 to checkpoint 3: 4.650 s
From checkpoint 3 to checkpoint 4: 4.750 s
From checkpoint 4 to checkpoint 5: 4.750 s
From checkpoint 5 to checkpoint 6: 3.650 s
From checkpoint 6 to checkpoint 7: 2.750 s

Average time: 3.964 s

---------------------------------------------



Qd = diag([
    50  50  100  ...   % x y z
    10  10  10  ...   % vx vy vz
    200  200  20  ...   % phi theta psi
    10  10  10         % wx wy wz
]);


Rd = 1*eye(4);

3333333333333333333333333333333333333333333333333333333333333
---------------------------------------------
   Quadcopter exercise - Simulation Report
---------------------------------------------

Full state vector: available
Number of checkpoints reached: 7/7
Payload: 0 kg

Timing:
-------
From initial pos. to checkpoint 1: 3.000 s
From checkpoint 1 to checkpoint 2: 3.900 s
From checkpoint 2 to checkpoint 3: 4.400 s
From checkpoint 3 to checkpoint 4: 4.500 s
From checkpoint 4 to checkpoint 5: 4.500 s
From checkpoint 5 to checkpoint 6: 3.300 s
From checkpoint 6 to checkpoint 7: 2.750 s

Average time: 3.764 s

---------------------------------------------



Qd = diag([
    50  50  100  ...   % x y z
    10  10  10  ...   % vx vy vz
    200 200 20  ...   % phi theta psi
    10  10  10         % wx wy wz
]);


Rd = 0.05*eye(4);

444444444444444444444444444444444444444444444444
---------------------------------------------
   Quadcopter exercise - Simulation Report
---------------------------------------------

Full state vector: available
Number of checkpoints reached: 7/7
Payload: 0 kg

Timing:
-------
From initial pos. to checkpoint 1: 1.550 s
From checkpoint 1 to checkpoint 2: 2.100 s
From checkpoint 2 to checkpoint 3: 2.250 s
From checkpoint 3 to checkpoint 4: 2.350 s
From checkpoint 4 to checkpoint 5: 2.350 s
From checkpoint 5 to checkpoint 6: 2.000 s
From checkpoint 6 to checkpoint 7: 1.400 s

Average time: 2.000 s

---------------------------------------------







%%%%% INTEGRAL CONTROL TUNING %%%%%%%%%%%%%%%


Qd = diag([
    10  10  10   ...   % Extra three states
    50  50  100  ...   % x y z
    10  10  10  ...   % vx vy vz
    200 200 20  ...   % phi theta psi
    10  10  10         % wx wy wz
]);


Rd = 0.05*eye(4);
00000000000000000000000000000000000000000000000000000000

CRASH :(


Qd = diag([
    1  1  1   ...   % Extra three states
    50  50  100  ...   % x y z
    10  10  10  ...   % vx vy vz
    200 200 20  ...   % phi theta psi
    10  10  10         % wx wy wz
]);


Rd = 0.05*eye(4);

111111111111111111111111111111111111111111111111111111111

Timing:
-------
From initial pos. to checkpoint 1: 2.050 s
From checkpoint 1 to checkpoint 2: 3.029 s
From checkpoint 2 to checkpoint 3: 3.300 s
From checkpoint 3 to checkpoint 4: 3.350 s
From checkpoint 4 to checkpoint 5: 3.350 s
From checkpoint 5 to checkpoint 6: 3.033 s
From checkpoint 6 to checkpoint 7: 1.900 s

Average time: 2.859 s

---------------------------------------------



Qd = diag([
    1  1  1   ...   % Extra three states
    50  50  100  ...   % x y z
    10  10  10  ...   % vx vy vz
    200 200 20  ...   % phi theta psi
    10  10  10         % wx wy wz
]);


Rd = 0.005*eye(4);

22222222222222222222222222222222222222222222222222222
---------------------------------------------
   Quadcopter exercise - Simulation Report
---------------------------------------------

Full state vector: available
Number of checkpoints reached: 7/7
Payload: 0 kg

Timing:
-------
From initial pos. to checkpoint 1: 1.850 s
From checkpoint 1 to checkpoint 2: 2.975 s
From checkpoint 2 to checkpoint 3: 3.300 s
From checkpoint 3 to checkpoint 4: 3.327 s
From checkpoint 4 to checkpoint 5: 3.327 s
From checkpoint 5 to checkpoint 6: 3.000 s
From checkpoint 6 to checkpoint 7: 1.600 s

Average time: 2.769 s

---------------------------------------------



%}