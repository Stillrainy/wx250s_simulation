%% wx250s Robot MATLAB Example
% This script demonstrates how to use the wx250s MATLAB class
% with the Robotics System Toolbox

clear; clc; close all;

%% Initialize Robot
fprintf('Initializing wx250s robot...\n');

try
    % Create robot object
    robot = wx250s();
    fprintf('Robot loaded successfully!\n');
    fprintf('Degrees of Freedom: %d\n', robot.DoF);
    
    % Display joint limits
    fprintf('\nJoint Limits:\n');
    for i = 1:robot.DoF
        fprintf('Joint %d: [%.2f, %.2f] rad\n', i-1, ...
            robot.joint_limits(i,1), robot.joint_limits(i,2));
    end
    
catch ME
    fprintf('Error loading robot: %s\n', ME.message);
    return;
end

%% Test Basic Functionality

% Go to home position
fprintf('\nMoving to home position...\n');
robot.goToHomePose();
home_positions = robot.getJointPositions();
fprintf('Home joint positions: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n', ...
    home_positions);

% Get end-effector pose at home
home_pose = robot.getEePose();
fprintf('\nEnd-effector pose at home:\n');
disp(home_pose);

%% Test Joint Movement

% Set some joint positions
joint_positions = deg2rad([0, -35, 60, 0, 65, 0]);
fprintf('\nSetting joint positions: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n', ...
    joint_positions);

success = robot.setJointPositions(joint_positions);
if success
    fprintf('Joint positions set successfully!\n');
    current_positions = robot.getJointPositions();
    fprintf('Current joint positions: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n', ...
        current_positions);
    
    % Get new end-effector pose
    new_pose = robot.getEePose();
    fprintf('\nNew end-effector pose:\n');
    disp(new_pose);
else
    fprintf('Failed to set joint positions!\n');
end

%% Visualize Robot

fprintf('\nVisualizing robot...\n');
robot.showRobot();
