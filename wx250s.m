classdef wx250s < handle
    % wx250s - MATLAB class for controlling wx250s robot using Robotics System Toolbox
    % 
    % This class provides an interface to the wx250s robot similar to the Python version,
    % using MATLAB's Robotics System Toolbox for URDF loading and forward kinematics.
    
    properties (Access = private)
        robot           % rigidBodyTree object
        joint_positions % Current joint positions
    end
    
    properties (Access = public)
        DoF             % Degrees of freedom
        joint_limits    % Joint limits [lower, upper]
        tool_T_tip_offset % Tool tip offset transformation
    end
    
    methods
        function obj = wx250s(urdf_path)
            % Constructor - Load URDF and initialize robot
            %
            % Parameters:
            %   urdf_path (optional): Path to URDF file (default: 'assets/wx250s.urdf')
            
            if nargin < 1
                urdf_path = 'assets/wx250s.urdf';
            end
            
            % Load URDF using importrobot
            obj.robot = importrobot(urdf_path);
            
            % Set degrees of freedom
            obj.DoF = 6;
            
            % Initialize joint positions
            obj.joint_positions = zeros(obj.DoF, 1);
            
            % Initialize tool tip offset as identity matrix
            obj.tool_T_tip_offset = eye(4);
            
            % Extract joint limits
            obj.extractJointLimits();
        end
        
        function success = setJointPositions(obj, joint_positions)
            % Set joint positions with limit checking
            %
            % Parameters:
            %   joint_positions: Vector of joint positions
            %
            % Returns:
            %   success: Boolean indicating if positions were set successfully
            
            if obj.checkInputs(joint_positions)
                obj.joint_positions = joint_positions(:);
                success = true;
            else
                success = false;
            end
        end
        
        function positions = getJointPositions(obj)
            % Get current joint positions
            %
            % Returns:
            %   positions: Current joint positions as column vector
            
            positions = obj.joint_positions;
        end
        
        function pose = getEePose(obj)
            % Get end-effector pose using forward kinematics
            %
            % Returns:
            %   pose: 4x4 homogeneous transformation matrix
            
            % Create configuration structure for rigidBodyTree
            config = homeConfiguration(obj.robot);
            
            % Set joint values
            for i = 1:min(length(config), obj.DoF)
                config(i).JointPosition = obj.joint_positions(i);
            end
            
            % Get transformation to end-effector
            % Find the end-effector body (typically the last body or gripper link)
            ee_body_name = obj.findEndEffectorBody();
            pose = getTransform(obj.robot, config, ee_body_name);
        end
        
        function goToHomePose(obj)
            % Move robot to home position (all joints at zero)
            
            obj.setJointPositions(zeros(obj.DoF, 1));
        end
        
        function showRobot(obj)
            % Visualize the robot in current configuration
            
            % Create configuration structure
            config = homeConfiguration(obj.robot);
            
            % Set joint values
            for i = 1:min(length(config), obj.DoF)
                config(i).JointPosition = obj.joint_positions(i);
            end
            
            % Show robot
            figure;
            show(obj.robot, config);
            title('wx250s Robot Configuration');
            axis equal;
            grid on;
        end
    end
    
    methods (Access = private)
        function extractJointLimits(obj)
            % Extract joint limits from rigidBodyTree
            
            % Get home configuration to determine number of joints
            config = homeConfiguration(obj.robot);
            num_joints = length(config);
            
            % Initialize joint limits matrix
            obj.joint_limits = zeros(obj.DoF, 2);
            
            % Extract limits for each joint up to DoF
            for i = 1:min(num_joints, obj.DoF)
                if i <= numel(obj.robot.Bodies)
                    body = obj.robot.Bodies{i};
                    joint = body.Joint;
                    
                    if ~isempty(joint.PositionLimits)
                        obj.joint_limits(i, :) = joint.PositionLimits;
                    else
                        % Default limits if not specified
                        obj.joint_limits(i, :) = [-pi, pi];
                    end
                else
                    % Default limits for joints beyond available bodies
                    obj.joint_limits(i, :) = [-pi, pi];
                end
            end
        end
        
        function valid = checkInputs(obj, joint_positions)
            % Check if joint positions are valid
            %
            % Parameters:
            %   joint_positions: Vector of joint positions
            %
            % Returns:
            %   valid: Boolean indicating if inputs are valid
            
            valid = true;
            
            % Check length
            if length(joint_positions) ~= obj.DoF
                warning('Input joint positions length %d does not match DoF %d.', ...
                    length(joint_positions), obj.DoF);
                valid = false;
                return;
            end
            
            % Check joint limits
            joint_positions = joint_positions(:);
            lower_limits = obj.joint_limits(:, 1);
            upper_limits = obj.joint_limits(:, 2);
            
            % Check lower limits
            below_limits = find(joint_positions < lower_limits);
            if ~isempty(below_limits)
                for idx = below_limits'
                    warning('Joint %d position %.2f is below lower limit %.2f.', ...
                        idx-1, joint_positions(idx), lower_limits(idx));
                end
                valid = false;
            end
            
            % Check upper limits
            above_limits = find(joint_positions > upper_limits);
            if ~isempty(above_limits)
                for idx = above_limits'
                    warning('Joint %d position %.2f is above upper limit %.2f.', ...
                        idx-1, joint_positions(idx), upper_limits(idx));
                end
                valid = false;
            end
        end
        
        function ee_body_name = findEndEffectorBody(obj)
            % Find the end-effector body name
            %
            % Returns:
            %   ee_body_name: Name of the end-effector body
            
            % Get body names using the correct MATLAB syntax
            num_bodies = numel(obj.robot.Bodies);
            body_names = cell(num_bodies, 1);
            
            for i = 1:num_bodies
                body_names{i} = obj.robot.Bodies{i}.Name;
            end
            
            % Search for gripper or end-effector related names
            ee_patterns = {'gripper', 'ee_', 'end_effector', 'tool'};
            
            for pattern = ee_patterns
                matching_bodies = contains(body_names, pattern{1}, 'IgnoreCase', true);
                if any(matching_bodies)
                    matching_indices = find(matching_bodies);
                    ee_body_name = body_names{matching_indices(end)}; % Take the last match
                    return;
                end
            end
            
            % If no specific end-effector found, use the last body
            if ~isempty(body_names)
                ee_body_name = body_names{end};
            else
                error('No suitable end-effector body found in robot model');
            end
        end
    end
end