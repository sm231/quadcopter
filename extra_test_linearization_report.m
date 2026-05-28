close all
clc

%%
set(groot, 'defaultAxesTickLabelInterpreter', 'latex'); 
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultTextInterpreter', 'latex');

set(groot, 'defaultAxesFontSize', 20);

% Set global Font Size for Text (includes X/Y Labels and Titles)
set(groot, 'defaultTextFontSize', 20);

% Set global Font Size for Legends
set(groot, 'defaultLegendFontSize', 14);

set(groot, 'defaultLineLineWidth', 2);
%% Simulation settings

t_final = 5;  % seconds
dt_plot = 0.01;  % resolution
t = (0:dt_plot:t_final)';

n_steps = length(t);

%% Define the deviation input Delta u

du = zeros(n_steps,4);

for i = 1:n_steps
    if t(i) >= 1
        du(i,:) = [5 5 5 5];
    else
        du(i,:) = [0 0 0 0];
    end
end

%% Simulate the linearized model
%
% Linear model:
%   Delta x_dot = A Delta x + B Delta u
%   Delta y     = C Delta x + D Delta u
%
% Initial condition:
%   x(0) = x_eq
% so
%   Delta x(0) = 0

sys_linear = ss(A,B,C,D);

dx0 = zeros(12,1);

[dy_linear, t_linear, dx_linear] = lsim(sys_linear, du, t, dx0);

% Convert linear output deviation back to original output variable:
%
%   y = y_eq + Delta y

y_linear = dy_linear + repmat(y_eq', n_steps, 1);

%% Simulate the nonlinear model
%
% Nonlinear model:
%   x_dot = xi(x,u)
%   y     = C x
%
% The nonlinear model needs the original input:
%
%   u = u_eq + Delta u

x0_nonlinear = x_eq;

[t_nonlinear, x_nonlinear] = ode45( ...
    @(t_current, x_current) nonlinear_quadcopter_rhs( ...
        t_current, x_current, u_eq, ...
        m, L, k, b, g, kd, Ixx, Iyy, Izz, cm), ...
    t, x0_nonlinear);

% Output of nonlinear model:
% y = [x y z phi theta psi]'

y_nonlinear = [x_nonlinear(:,1), ...
               x_nonlinear(:,2), ...
               x_nonlinear(:,3), ...
               x_nonlinear(:,7), ...
               x_nonlinear(:,8), ...
               x_nonlinear(:,9)];

%% Plot input perturbation

figure
plot(t, du(:,1), 'LineWidth', 1.5)
hold on
plot(t, du(:,2), '--', 'LineWidth', 1.5)
plot(t, du(:,3), '-.', 'LineWidth', 1.5)
plot(t, du(:,4), ':', 'LineWidth', 1.5)
grid on
xlabel('Time [s]')
ylabel('\Delta u_i [V^2]')
title('Input perturbation applied to both models')
legend('$\Delta u_1$','$\Delta u_2$','$\Delta u_3$','$\Delta u_4$', ...
       'Location','best')

%% Plot output comparison

output_names = {'x [m]', 'y [m]', 'z [m]', ...
                '\phi [rad]', '\theta [rad]', '\psi [rad]'};

figure

for i = 1:6
    subplot(3,2,i)
    plot(t_nonlinear, y_nonlinear(:,i), 'LineWidth', 1.5)
    hold on
    plot(t_linear, y_linear(:,i), '--', 'LineWidth', 1.5)
    grid on
    xlabel('Time [s]')
    ylabel(output_names{i})
    title(['Output ', output_names{i}])
    legend('Nonlinear model', 'Linear model', 'Location', 'best')
end

sgtitle('Validation of linearized quadcopter model')

%% Plot output error

output_error = y_nonlinear - y_linear;

figure

for i = 1:6
    subplot(3,2,i)
    plot(t, output_error(:,i), 'LineWidth', 1.5)
    grid on
    xlabel('Time [s]')
    ylabel(['Error in ', output_names{i}])
    title(['Nonlinear - linear: ', output_names{i}])
end

sgtitle('Difference between nonlinear and linear model outputs')

%% Print maximum absolute errors

max_abs_error = max(abs(output_error), [], 1);

disp('Maximum absolute output errors over 5 seconds:')
disp('Columns: x, y, z, phi, theta, psi')
disp(max_abs_error)

%% Local nonlinear model function

function x_dot = nonlinear_quadcopter_rhs(t_current, x_current, u_eq, ...
                                          m, L, k, b, g, kd, ...
                                          Ixx, Iyy, Izz, cm)

    % Step signal in deviation input
    if t_current >= 1
        du_current = [5; 5; 5; 5];
    else
        du_current = [0; 0; 0; 0];
    end

    % Convert from deviation input to original input
    u_current = u_eq + du_current;

    % The inputs are squared voltages:
    u1 = u_current(1);
    u2 = u_current(2);
    u3 = u_current(3);
    u4 = u_current(4);

    % State vector:
    % x = [x y z vx vy vz phi theta psi omega_x omega_y omega_z]'

    vx = x_current(4);
    vy = x_current(5);
    vz = x_current(6);

    phi   = x_current(7);
    theta = x_current(8);
    psi   = x_current(9);

    omega_x = x_current(10);
    omega_y = x_current(11);
    omega_z = x_current(12);

    total_input = u1 + u2 + u3 + u4;

    x_dot = zeros(12,1);

    % Translational kinematics
    x_dot(1) = vx;
    x_dot(2) = vy;
    x_dot(3) = vz;

    % Translational dynamics
    x_dot(4) = -(kd/m)*vx ...
               + (k*cm/m)*(sin(psi)*sin(phi) ...
               + cos(psi)*cos(phi)*sin(theta))*total_input;

    x_dot(5) = -(kd/m)*vy ...
               + (k*cm/m)*(cos(phi)*sin(psi)*sin(theta) ...
               - cos(psi)*sin(phi))*total_input;

    x_dot(6) = -(kd/m)*vz ...
               - g ...
               + (k*cm/m)*(cos(theta)*cos(phi))*total_input;

    % Attitude kinematics
    x_dot(7) = omega_x ...
               + omega_y*sin(phi)*tan(theta) ...
               + omega_z*cos(phi)*tan(theta);

    x_dot(8) = omega_y*cos(phi) ...
               - omega_z*sin(phi);

    x_dot(9) = omega_y*(sin(phi)/cos(theta)) ...
               + omega_z*(cos(phi)/cos(theta));

    % Rotational dynamics
    x_dot(10) = (L*k*cm/Ixx)*(u1 - u3) ...
                - ((Iyy - Izz)/Ixx)*omega_y*omega_z;

    x_dot(11) = (L*k*cm/Iyy)*(u2 - u4) ...
                - ((Izz - Ixx)/Iyy)*omega_x*omega_z;

    x_dot(12) = (b*cm/Izz)*(u1 - u2 + u3 - u4) ...
                - ((Ixx - Iyy)/Izz)*omega_x*omega_y;
end