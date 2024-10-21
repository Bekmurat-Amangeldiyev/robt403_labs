%% Task 1 
function workspace_3DOF_planar(a1, a2, a3, num_points)
    % Inputs:
    % a1, a2, a3: Lengths of the links
    % num_points: Resolution of the grid for theta1, theta2, and theta3
    
    % Define joint angle ranges (from -pi/2 to pi/2 for all joints)
    theta1_range = linspace(-pi/2, pi/2, num_points);
    theta2_range = linspace(-pi/2, pi/2, num_points);
    theta3_range = linspace(-pi/2, pi/2, num_points);
    
    % Preallocate arrays to store x and y coordinates of end-effector
    num_total_points = num_points^3;
    x_positions = zeros(1, num_total_points);
    y_positions = zeros(1, num_total_points);
    
    % Initialize index for storing positions
    index = 1;
    
    % Loop through all combinations of theta1, theta2, and theta3
    for theta1 = theta1_range
        for theta2 = theta2_range
            for theta3 = theta3_range
                % Calculate forward kinematics for current joint angles
                % x and y are positions of the end-effector in the 2D plane
                x = a1 * cos(theta1) + a2 * cos(theta1 + theta2) + a3 * cos(theta1 + theta2 + theta3);
                y = a1 * sin(theta1) + a2 * sin(theta1 + theta2) + a3 * sin(theta1 + theta2 + theta3);
                
                % Store the position at the current index
                x_positions(index) = x;
                y_positions(index) = y;
                
                % Increment the index
                index = index + 1;
            end
        end
    end
    
    % Plot the workspace
    figure;
    scatter(x_positions, y_positions, 1, 'filled');
    title('Workspace of 3DOF Planar Manipulator');
    xlabel('X Position');
    ylabel('Y Position');
    axis equal; % Make sure the aspect ratio is equal
    grid on;
end
%%
a1 = 13.7; % Length of link 1
a2 = 13.7; % Length of link 2
a3 = 4.5; % Length of link 3 (previously named d)
num_points = 50; % Resolution of the workspace calculation

workspace_3DOF_planar(a1, a2, a3, num_points);

%% Task 2
%% Calculate Cubic Polynomial Coefficients
function [a0, a1, a2, a3] = cubic_coefficients(q_initial, q_final, t_initial, t_final)
    % Inputs:
    % q_initial: Initial joint angle
    % q_final: Final joint angle
    % t_initial: Initial time
    % t_final: Final time
    
    % Time duration
    T = t_final - t_initial;
    
    % Cubic polynomial coefficients
    a0 = q_initial;
    a1 = 0;
    a2 = (3 / T^2) * (q_final - q_initial);
    a3 = (-2 / T^3) * (q_final - q_initial);
end

%% Calculate Joint Trajectory Using Cubic Polynomial
 function q_t = cubic_trajectory(a0, a1, a2, a3, t)
     % Inputs:
     % a0, a1, a2, a3: Cubic polynomial coefficients
     % t: Time vector or scalar at which to evaluate the trajectory
 
     % Calculate joint angle at time t with element-wise operations
     q_t = a0 + a1 * t + a2 * t.^2 + a3 * t.^3;  % Use .^ for element-wise power
 end

%% Implementing and Plotting Joint Trajectories
% Example usage in radians:
q1_initial = deg2rad(0); % Initial joint angle for q1 (in radians)
q1_final = deg2rad(45);    % Final joint angle for q1 (in radians)

q2_initial = deg2rad(0); % Initial joint angle for q2 (in radians)
q2_final = deg2rad(-45);    % Final joint angle for q2 (in radians)

q3_initial = deg2rad(90); % Initial joint angle for q3 (in radians)
q3_final = deg2rad(45);    % Final joint angle for q3 (in radians)

t_initial = 0;    % Initial time
t_final = 10;     % Final time (seconds)

% Calculate cubic coefficients for q1, q2, and q3
[a0_q1, a1_q1, a2_q1, a3_q1] = cubic_coefficients(q1_initial, q1_final, t_initial, t_final);
[a0_q2, a1_q2, a2_q2, a3_q2] = cubic_coefficients(q2_initial, q2_final, t_initial, t_final);
[a0_q3, a1_q3, a2_q3, a3_q3] = cubic_coefficients(q3_initial, q3_final, t_initial, t_final);

% Generate time vector
t = linspace(t_initial, t_final, 100); % 100 points for smoother trajectory

% Calculate joint trajectories for q1, q2, and q3
q1_trajectory = cubic_trajectory(a0_q1, a1_q1, a2_q1, a3_q1, t);
q2_trajectory = cubic_trajectory(a0_q2, a1_q2, a2_q2, a3_q2, t);
q3_trajectory = cubic_trajectory(a0_q3, a1_q3, a2_q3, a3_q3, t);

% Plot combined joint trajectories with improved styling
figure;

% Plot q1, q2, and q3 with different colors and line styles
plot(t, q1_trajectory, 'b', 'LineWidth', 2); % q1 in blue
hold on;
plot(t, q2_trajectory, 'r', 'LineWidth', 2); % q2 in red
plot(t, q3_trajectory, 'g', 'LineWidth', 2); % q3 in green
hold off;

% Add labels, title, and grid
title('Cubic Trajectory for the 3-DOF Planar Manipulator', 'FontSize', 12);
xlabel('Time [s]', 'FontSize', 10);
ylabel('Joint Angles [rad]', 'FontSize', 10);
legend('Theta 1', 'Theta 3', 'Theta 5', 'Location', 'northeast');
grid on;

% Set axis limits for better visibility if needed (optional)
xlim([0 t_final]);
ylim([-1 2]); % Adjust according to your specific data range




%% Task 3 ANY CODE BELOW IS NOT USABLE
 
%% CODE sample 1
function [q1, q2, q3] = inverse_kinematics_with_orientation(x_target, y_target, theta_target, a1, a2, a3)
    % Inverse kinematics for 3DOF planar robot with orientation

    % Calculate the position of the wrist (end of second link)
    % Subtract the effect of the third link (orientation part)
    wrist_x = x_target - a3 * cos(theta_target);
    wrist_y = y_target - a3 * sin(theta_target);

    % Calculate q1 and q2 using the wrist position and geometric relationships
    % Distance from the base to the wrist
    r = sqrt(wrist_x^2 + wrist_y^2);

    % Check if the target is reachable
    if r > (a1 + a2) || r < abs(a1 - a2)
        error('Target position is out of reach.');
    end

    % Use the law of cosines to solve for q2
    cos_q2 = (r^2 - a1^2 - a2^2) / (2 * a1 * a2);
    q2 = atan2(sqrt(1 - cos_q2^2), cos_q2); % Elbow down configuration

    % Solve for q1 using the law of cosines
    beta = atan2(wrist_y, wrist_x); % Angle from origin to wrist position
    alpha = atan2(a2 * sin(q2), a1 + a2 * cos(q2)); % Angle to compensate for link lengths
    q1 = beta - alpha;

    % Calculate q3 using the target orientation
    q3 = theta_target - (q1 + q2); % Subtract q1 and q2 from the total orientation

    % Ensure q1, q2, and q3 are within their limits (optional)
    % Convert to degrees for clarity (optional)
    q1 = rad2deg(q1);
    q2 = rad2deg(q2);
    q3 = rad2deg(q3);
end
%%
% Link lengths
a1 = 13.7; % cm
a2 = 13.7; % cm
a3 = 4.5;  % cm

% Target position and orientation
x_target = 20; % cm
y_target = 10; % cm
theta_target = deg2rad(30); % Desired orientation in radians

% Calculate joint angles
[q1, q2, q3] = inverse_kinematics_with_orientation(x_target, y_target, theta_target, a1, a2, a3);

% Display results
fprintf('q1 = %.2f deg, q2 = %.2f deg, q3 = %.2f deg\n', q1, q2, q3);

%% CODE sample 2
% function plot_robot_trajectory(x_target, y_target, theta_target, a1, a2, a3)
%     % Plot the manipulator and its trajectory based on the inverse kinematics solution.
% 
%     % Compute the joint angles using inverse kinematics
%     [q1, q2, q3] = inverse_kinematics_with_orientation(x_target, y_target, theta_target, a1, a2, a3);
% 
%     % Convert angles back to radians
%     q1 = deg2rad(q1);
%     q2 = deg2rad(q2);
%     q3 = deg2rad(q3);
% 
%     % Calculate the positions of each joint and end-effector
%     x0 = 0; y0 = 0;  % Base
%     x1 = a1 * cos(q1); % Joint 1
%     y1 = a1 * sin(q1);
% 
%     x2 = x1 + a2 * cos(q1 + q2); % Joint 2
%     y2 = y1 + a2 * sin(q1 + q2);
% 
%     x3 = x2 + a3 * cos(q1 + q2 + q3); % End-effector (joint 3)
%     y3 = y2 + a3 * sin(q1 + q2 + q3);
% 
%     % Define the workspace of the manipulator for visualization
%     theta_range = linspace(-pi, pi, 100);
%     r_workspace = a1 + a2 + a3;
%     x_workspace = r_workspace * cos(theta_range);
%     y_workspace = r_workspace * sin(theta_range);
% 
%     % Plot the workspace
%     figure;
%     fill(x_workspace, y_workspace, [0.2 0.4 1], 'FaceAlpha', 0.5); % Workspace region
%     hold on;
% 
%     % Plot the manipulator's links
%     plot([x0, x1], [y0, y1], 'g', 'LineWidth', 3); % Link 1
%     plot([x1, x2], [y1, y2], 'b', 'LineWidth', 3); % Link 2
%     plot([x2, x3], [y2, y3], 'r', 'LineWidth', 3); % Link 3
% 
%     % Plot the joints
%     plot(x0, y0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k'); % Base joint
%     plot(x1, y1, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Joint 1
%     plot(x2, y2, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'b'); % Joint 2
%     plot(x3, y3, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % End-effector
% 
%     % Plot the end-effector trajectory (example as a circular arc)
%     theta_traj = linspace(0, pi, 50); % Example trajectory for demonstration
%     x_traj = x_target + 2 * cos(theta_traj);
%     y_traj = y_target + 2 * sin(theta_traj);
%     plot(x_traj, y_traj, 'r--', 'LineWidth', 2);
% 
%     % Mark the start and end points of the end-effector's trajectory
%     plot(x_traj(1), y_traj(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
%     plot(x_traj(end), y_traj(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
% 
%     % Add labels and title
%     title('Manipulator Configuration with End-Effector Orientation', 'FontSize', 12);
%     xlabel('X Position');
%     ylabel('Y Position');
%     legend({'Workspace', 'Link 1', 'Link 2', 'Link 3', 'End-effector Trajectory', 'Start/End Points'}, ...
%            'Location', 'best');
% 
%     axis equal;
%     grid on;
%     hold off;
% end
%%
% Define link lengths (example)
a1 = 13.7; % Length of link 1 (in cm)
a2 = 13.7; % Length of link 2 (in cm)
a3 = 4.5;  % Length of link 3 (in cm)

% Define target position and orientation
x_target = 4; % Target X position (in cm)
y_target = 4; % Target Y position (in cm)
theta_target = deg2rad(45); % Desired orientation in radians

% Call the function to plot the robot trajectory
plot_robot_trajectory(x_target, y_target, theta_target, a1, a2, a3);

%% CODE sample 3
function plot_robot_trajectory_in_workspace(a1, a2, a3, num_points, x_target, y_target, theta_target)
    % Plot the workspace using the predefined function
    workspace_3DOF_planar(a1, a2, a3, num_points);
    hold on; % Keep the workspace plot for additional trajectories

    % Compute the joint angles using inverse kinematics
    [q1, q2, q3] = inverse_kinematics_with_orientation(x_target, y_target, theta_target, a1, a2, a3);
    fprintf('q1 = %.2f deg, q2 = %.2f deg, q3 = %.2f deg\n', q1, q2, q3);
    % Convert angles to radians
    q1 = deg2rad(q1);
    q2 = deg2rad(q2);
    q3 = deg2rad(q3);

    % Calculate the positions of each joint and end-effector
    x0 = 0; y0 = 0;  % Base
    x1 = a1 * cos(q1); % Joint 1
    y1 = a1 * sin(q1);
    
    x2 = x1 + a2 * cos(q1 + q2); % Joint 2
    y2 = y1 + a2 * sin(q1 + q2);
    
    x3 = x2 + a3 * cos(q1 + q2 + q3); % End-effector (joint 3)
    y3 = y2 + a3 * sin(q1 + q2 + q3);

    % Plot the manipulator's links
    plot([x0, x1], [y0, y1], 'g', 'LineWidth', 3); % Link 1
    plot([x1, x2], [y1, y2], 'b', 'LineWidth', 3); % Link 2
    plot([x2, x3], [y2, y3], 'r', 'LineWidth', 3); % Link 3

    % Plot the joints
    plot(x0, y0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k'); % Base joint
    plot(x1, y1, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Joint 1
    plot(x2, y2, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'b'); % Joint 2
    plot(x3, y3, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % End-effector

    % Plot the end-effector trajectory (example as a circular arc)
    theta_traj = linspace(0, pi, 50); % Example trajectory for demonstration
    x_traj = x_target + 2 * cos(theta_traj);
    y_traj = y_target + 2 * sin(theta_traj);
    plot(x_traj, y_traj, 'r--', 'LineWidth', 2);

    % Mark the start and end points of the end-effector's trajectory
    plot(x_traj(1), y_traj(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    plot(x_traj(end), y_traj(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

    % Add labels and title
    title('Manipulator Configuration with End-Effector Orientation', 'FontSize', 12);
    xlabel('X Position');
    ylabel('Y Position');
    legend({'Workspace', 'Link 1', 'Link 2', 'Link 3', 'End-effector Trajectory', 'Start/End Points'}, ...
           'Location', 'best');
    
    axis equal;
    grid on;
    hold off;
end

%% Example usage:
a1 = 13.7; % Length of link 1 (in cm)
a2 = 13.7; % Length of link 2 (in cm)
a3 = 4.5;  % Length of link 3 (in cm)
num_points = 50; % Resolution of the workspace calculation
x_target = 20; % Target X position (in cm)
y_target = 20; % Target Y position (in cm)
theta_target = deg2rad(45); % Desired orientation in radians

plot_robot_trajectory_in_workspace(a1, a2, a3, num_points, x_target, y_target, theta_target);


%% Task 4




%% Task 5

function plot_via_points_trajectory(via_points, time_points, a1, a2, a3)
    % Inputs:
    % via_points: N x 2 matrix where each row is [x, y] Cartesian coordinates of via points
    % time_points: 1 x N vector of time instances for each via point
    % a1, a2, a3: Lengths of the links
    
    % Initialize number of via points
    N = size(via_points, 1);
    
    % Compute inverse kinematics for each via point (Cartesian to Joint Space)
    joint_angles = zeros(N, 3);
    for i = 1:N
        [joint_angles(i, 1), joint_angles(i, 2), joint_angles(i, 3)] = inverse_kinematics_3DOF(via_points(i, 1), via_points(i, 2), a1, a2, a3);
    end
    
    % Joint-space cubic trajectory generation
    dt = 0.01; % Time step for interpolation
    t = time_points(1):dt:time_points(end); % Time vector
    q_traj = zeros(length(t), 3); % Joint-space trajectory
    
    for j = 1:3 % For each joint (q1, q2, q3)
        q_traj(:, j) = cubic_trajectory(joint_angles(:, j), time_points, t);
    end
    
    % Compute the Cartesian trajectory from joint trajectories
    x_traj = zeros(1, length(t));
    y_traj = zeros(1, length(t));
    
    for k = 1:length(t)
        [x_traj(k), y_traj(k)] = forward_kinematics_3DOF(q_traj(k, 1), q_traj(k, 2), q_traj(k, 3), a1, a2, a3);
    end
    
    % Plot the via points in the Cartesian space
    figure;
    hold on;
    scatter(via_points(:, 1), via_points(:, 2), 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'r');
    
    % Plot the Cartesian trajectory
    plot(x_traj, y_traj, 'b', 'LineWidth', 1.5);
    title('End-Effector Path with Via Points');
    xlabel('X Position (cm)');
    ylabel('Y Position (cm)');
    axis equal;
    grid on;
    hold off;
end
%%
%  function q_traj = cubic_trajectory(via_angles, time_points, t)
%      % Generates cubic trajectories between via angles in joint space
%      N = length(via_angles);
%      q_traj = zeros(size(t));
% 
%      for i = 1:N-1
%          % Get initial and final conditions
%          q0 = via_angles(i);
%          qf = via_angles(i+1);
%          t0 = time_points(i);
%          tf = time_points(i+1);
% 
%          % Coefficients for cubic trajectory: a0 + a1*t + a2*t^2 + a3*t^3
%          a0 = q0;
%          a1 = 0; % Assuming zero initial velocity
%          a2 = (3/(tf-t0)^2) * (qf - q0);
%          a3 = (-2/(tf-t0)^3) * (qf - q0);
% 
%         % Generate the cubic trajectory for this segment
%         idx = t >= t0 & t <= tf;
%         tau = t(idx) - t0; % Relative time
%         q_traj(idx) = a0 + a1 * tau + a2 * tau.^2 + a3 * tau.^3;
%     end
% end
%%
function [theta1, theta2, theta3] = inverse_kinematics_3DOF(x, y, a1, a2, a3)
    % Simple inverse kinematics assuming 3DOF planar robot
    % Solve for joint angles theta1, theta2, and theta3 using geometric approach
    
    r = sqrt(x^2 + y^2);
    cos_theta3 = (r^2 - a1^2 - a2^2) / (2 * a1 * a2);
    theta3 = acos(cos_theta3);
    
    phi = atan2(y, x);
    psi = atan2(a2 * sin(theta3), a1 + a2 * cos(theta3));
    
    theta1 = phi - psi;
    theta2 = atan2(y - a1 * sin(theta1), x - a1 * cos(theta1)) - theta1;
end
%%
function [x, y] = forward_kinematics_3DOF(theta1, theta2, theta3, a1, a2, a3)
    % Compute the Cartesian coordinates from joint angles (forward kinematics)
    x = a1 * cos(theta1) + a2 * cos(theta1 + theta2) + a3 * cos(theta1 + theta2 + theta3);
    y = a1 * sin(theta1) + a2 * sin(theta1 + theta2) + a3 * sin(theta1 + theta2 + theta3);
end

%% 
[x, y] = forward_kinematics_3DOF(x_target, y_target, theta_target, a1, a2, a3);

% Display results
fprintf('x = %.2f , y = %.2f \n', x, y);


%%
% Define via points in Cartesian space
via_points = [10, 10; 15, 5; 20, 10; 25, 5];

% Define time instances for each via point
time_points = [0, 2, 4, 6];

% Link lengths of the 3DOF robot
a1 = 13.7;
a2 = 13.7;
a3 = 4.5;

% Call the function to plot the trajectory
plot_via_points_trajectory(via_points, time_points, a1, a2, a3);

