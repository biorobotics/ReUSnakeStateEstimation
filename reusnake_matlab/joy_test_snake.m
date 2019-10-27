%% 2019-10-24
% reusnake gait implementation
% 
%

%% Add the path of all the dependency files and init parameters
addpath(genpath(fullfile(pwd,'hebi')));
% Add the path of all manpulator function
addpath(genpath(fullfile(pwd,'ModernRobotics')));


%% setup robot, visual figure and so on
setup_script;


%% control loop starts
t = 0;   % a time parameter

% sidewinding parameters
wS = 0.5;
wT= 4;
A_even = 1.1;
A_odd = .9;
delta = pi/4; % right
% delta = -pi/4; % left
beta_odd = 0;
beta_even = 0;

% amp_mult = linspace(0,1,numModules); % conical
amp_mult = ones(1,numModules); % straight

% TODO: add a bunch of ROS publisher
% Just use ROS as a visualizer
rosinit
joint_state_msg = rosmessage('sensor_msgs/JointState');
joint_state_pub = rospublisher('/reusnake/joint_state','sensor_msgs/JointState');
for i=1:numModules
    joint_state_msg.Name{i} = strcat('joint',int2str(i));
end
joint_state_msg.Position = zeros(1,numModules);
joint_state_msg.Velocity = zeros(1,numModules);
joint_state_msg.Effort = zeros(1,numModules);
% send(joint_state_pub, joint_state_msg);
% How to send TF
% tfStampedMsg = rosmessage('geometry_msgs/TransformStamped');
% tfStampedMsg.ChildFrameId = 'wheel';
% tfStampedMsg.Header.FrameId = 'robot_base';
% tfStampedMsg.Header.Stamp = rostime('now');
% sendTransform(tftree, tfStampedMsg)

% to calculate virtual chassis
init_virtualChassis = 0;
last_virtualChassis = eye(4);

while on_button ~= 1
    % joy read
    [axes, buttons, pov] = read( joy );
    on_button = buttons(9);
    if buttons == x
        t = t + 0.01;
    elseif buttons == y
        t = t - 0.01;
    end
    %this gets the feedback from the bot
%     fbk =
%                   time: 1.572e+09 [s]
%            positionCmd: [1x14] [rad]
%            velocityCmd: [1x14] [rad/s]
%              effortCmd: [1x14] [Nm|N]
%               position: [1x14] [rad]
%               velocity: [1x14] [rad/s]
%                 effort: [1x14] [Nm|N]
%                 accelX: [1x14] [m/s^2]
%                 accelY: [1x14] [m/s^2]
%                 accelZ: [1x14] [m/s^2]
%                  gyroX: [1x14] [rad/s]
%                  gyroY: [1x14] [rad/s]
%                  gyroZ: [1x14] [rad/s]
%           motorCurrent: [1x14] [A]
%         windingCurrent: [1x14] [A]
%     ambientTemperature: [1x14] [C]
%     windingTemperature: [1x14] [C]
%                voltage: [1x14] [V]
    fbk = [];
    while isempty(fbk)
        fbk = bot.getNextFeedback(); % get current feedback
        if isempty(fbk)
            disp('No fbk');
        end
    end
    lastFbk = fbk;
    
    % the order follows the order in Scope
%     fbk.position % print position to debug 
    for i=1:numModules
        joint_state_msg.Position(i) = fbk.position(i);
    end
    % send fbk to ros
    send(joint_state_pub, joint_state_msg);
    
    % flip position?
    g = snake_fk_cal(fbk.position);
    
    %% average rotation
    % convert rotations to quaternion
    R_list = g(1:3,1:3,:);
    q_list = quaternion(R_list, 'rotmat', 'frame');
    % find average quaternion
    quatAverage = meanrot(q_list);
    R_average = rotmat(quatAverage, 'frame');
    
    %% virtual chassis (from old code in Julian's demo)
    xyz_pts = squeeze( g(1:3,4,:) )';
    % Find and shift to the center of mass.
    CoM = mean(xyz_pts);
    xyz_pts = xyz_pts - repmat(CoM,size(xyz_pts,1),1);
    % Take the SVD of the zero-meaned positions
    [~, ~, V] = svd( xyz_pts );
    if (init_virtualChassis == 1)
        % use last virtual chassis to help current virtual chassis
        % estimation
        lastVC_R = last_virtualChassis(1:3,1:3);     
        virtualChassis_R = V;
        % Make sure the X-axis isn't flipped
        if dot( virtualChassis_R(:,1), lastVC_R(:,1) ) < 0
            virtualChassis_R(:,1) = -virtualChassis_R(:,1);
        end
        % Make sure the Y-axis isn't flipped
        if dot( virtualChassis_R(:,2), lastVC_R(:,2) ) < 0
            virtualChassis_R(:,2) = -virtualChassis_R(:,2);
        end
        % Use the cross product of the first 2 axes to define the 3rd.
        % Ensures right-handed coordinates.
        virtualChassis_R(:,3) = cross( virtualChassis_R(:,1), ...
                                     virtualChassis_R(:,2) );
    else
        % Account for possible sign flips in SVD.
        % Point the 1st principal moment toward the head module
        if dot( V(:,1), xyz_pts(1,:) ) < 0
            V(:,1) = -V(:,1);
        end
        % Use the cross product of the first 2 axes to define the 3rd.
        % Ensures right-handed coordinates.
        V(:,3) = cross(V(:,1), V(:,2));
        % Permute the axes and eigenvalues accordingly.
        virtualChassis_R = V;   
    end

    % Make the full Virtual Chassis transformation matrix
    virtualChassis = eye(4);
    % TODO: compare these two methods
%     virtualChassis(1:3,1:3) = virtualChassis_R;
    virtualChassis(1:3,1:3) = R_average;
    virtualChassis(1:3,4) = CoM';
    
    if (init_virtualChassis == 0)
        last_virtualChassis = virtualChassis;
        init_virtualChassis = 1;
    end
    %% visualize robot
    snake_fk_plot(g,virtualChassis,fk_axh,0);

    %% control the robot
    for i = 1:numModules
        if mod(i,2) % it is odd
            commandedAngles(i) = beta_odd + A_odd*amp_mult(i)*sin(wS*i-wT*t);
        else % is even
            commandedAngles(i) = beta_even + A_even*amp_mult(i)*sin(wS*i-wT*t + delta);
        end
    end
    if buttons(1) == 1
        commandedAngles = zeros(1,numModules);
    end

    cmd.position = commandedAngles;
    
    bot.set(cmd);
    
 
    pause(0.005); % 200Hz 
end
clear all
close all
rosshutdown

















