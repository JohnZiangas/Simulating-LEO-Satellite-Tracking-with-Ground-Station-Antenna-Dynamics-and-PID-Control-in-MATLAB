%% LEO Satellite Tracking Simulation
% This program was developed as part of a semester assignment for the course Advanced Control Systems (ECE_ELE760) during the 2024-2025 academic year.
% University of the Peloponeese
% School of Electrical & Computer Engineering
% Ioannis Ziangas, Undergraduate Student
% Supervisor: Georgios Souliotis, Associate Professor

%% Clearing the command window and closing all figures
clc;           % Clear the command window
close all hidden; % Close all hidden figures
clear;         % Clear all variables from the workspace
close all;     % Close all open figures

%% Importing the Azimuth values from the Satellite Scenario
load('Azimuth_Values.mat', 'azim'); % Load the desired azimuth data from the file

%% Creating the transfer function of the motor
% Define motor parameters (example values for demonstration)
K_t = 0.45;    % Torque constant (Nm/A) - proportional to the torque produced per ampere
J = 0.12;      % Moment of inertia (kg*m^2) - resistance to angular acceleration
B = 0.06;      % Damping coefficient (Nm/(rad/s)) - friction-like resistance
R_a = 1.8;     % Armature resistance (Ohms) - resistance of the motor windings
L_a = 0.015;   % Armature inductance (H) - inductance of the motor windings
K_b = 0.45;    % Back EMF constant (V/(rad/s)) - proportional to speed-induced voltage

% Define the transfer function for the motor dynamics (antenna's azimuth axis)
num = K_t; % Numerator (torque constant)
den = [L_a*J, (R_a*J + L_a*B), (R_a*B + K_b*K_t)]; % Denominator of the motor transfer function
G_motor_azimuth = tf(num, den); % Create the transfer function
display(G_motor_azimuth); % Display the motor transfer function

% Scale the transfer function to ensure the DC gain is 1
scaling_factor = 1 / dcgain(G_motor_azimuth); % Compute scaling factor for unity DC gain
G_motor_scaled = scaling_factor * G_motor_azimuth; % Scaled transfer function

%% Creating the PID controller
% Define the PID controller gains
Kp = 0.9329;  % Proportional gain
Ki = 2.7358;  % Integral gain
Kd = 0;       % Derivative gain
G_PID = pid(Kp, Ki, Kd); % Create a PID controller with specified gains

% Uncomment the following line to automatically tune the PID controller (optional)
% G_PID = pidtune(G_motor_azimuth, 'PID');

% Define the closed-loop transfer function with the PID controller
G_closed_loop = feedback(G_PID * G_motor_azimuth, 1); % Closed-loop system
display(G_closed_loop); % Display the closed-loop transfer function

% Load the desired azimuth angle data (sinusoidal variation, for example)
desired_azimuth = azim; % Load the azimuth values

% Define the time vector for simulation
t = 1:1:length(desired_azimuth); % Time in seconds (sampled every second)

% Simulate the closed-loop response to the desired azimuth angle
[actual_azimuth, ~] = lsim(G_closed_loop, desired_azimuth, t); % Linear simulation

%% Plotting the results
% Define figure properties for publication-quality plotting
figure('Name', 'Antenna Azimuth Tracking Response', 'Units', 'Inches', 'Position', [1, 1, 6, 4]);

% Plot the actual azimuth (antenna response)
plot(t / 60, actual_azimuth, 'b-', 'LineWidth', 2); % Blue solid line for actual azimuth
hold on;

% Plot the desired azimuth (satellite movement)
plot(t / 60, desired_azimuth, 'r--', 'LineWidth', 2); % Red dashed line for desired azimuth

% Add grid and legend
grid on;
legend('Actual Azimuth (Antenna)', 'Desired Azimuth (Satellite)', ...
       'FontSize', 11, 'Location', 'Best'); % Add legend with appropriate size

% Add title and axis labels with proper formatting
title('Azimuth PID Controller', 'FontSize', 14, 'FontWeight', 'Bold');
xlabel('Time (Minutes)', 'FontSize', 12); % X-axis represents time in minutes
ylabel('Azimuth Angle (Degrees)', 'FontSize', 12); % Y-axis represents azimuth angle

% Improve axis appearance
set(gca, 'FontSize', 12, 'LineWidth', 1, 'Box', 'on', 'GridAlpha', 0.2); % Customize axis appearance

% Set tight axis limits for better visual balance
xlim([0, max(t) / 60]); % Limit X-axis to the maximum time (in minutes)
ylim([min(desired_azimuth) - 5, max(desired_azimuth) + 5]); % Limit Y-axis to the azimuth range

% Ensure figure looks good for printing or saving
set(gcf, 'Color', 'w'); % Set background to white for publication quality

%% Step Response Analysis
% Compute step response information for the closed-loop system
step_info = stepinfo(G_closed_loop);

% Define a time vector for step response simulation
t = 0:0.01:3; % Time vector for step response (0 to 3 seconds)

% Plot the step response for both closed-loop and open-loop systems
figure('Name', 'PID vs TF Step Response'); % Create a new figure
hold on;
stepplot(G_closed_loop, G_motor_scaled, t); % Plot step responses
grid on;

% Add labels, title, and legend with LaTeX formatting
xlabel("Time", 'Interpreter', 'latex', 'FontSize', 14); % X-axis label
ylabel("Amplitude", 'Interpreter', 'latex', 'FontSize', 14); % Y-axis label
title("Step Response of the Azimuth Control System", 'Interpreter', 'latex', 'FontSize', 16); % Title
legend("PID", "TF Open Loop", 'Interpreter', 'latex', 'Location', 'southeast', 'FontSize', 11); % Legend
hold off;
