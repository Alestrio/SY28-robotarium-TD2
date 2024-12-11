% Same script as leader_follower_with_plotting.m but with additional data saving. Two
% data sets will be saved, one saving the distance between connected robots
% through time, and another with the distance between the leader and goal
% location when the goal is "reached".

% Sean Wilson
% 07/2019

%% Experiment Constants
close all;
%Run the simulation for a specific number of iterations
iterations = 5000;

%% Set up the Robotarium object

N = 5;
initial_positions = generate_initial_conditions(N, 'Width', 1, 'Height', 1, 'Spacing', 0.3);
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);
net_utils = SY28_network_utils();
% This class is responsible for the network side of the project. 

fig = r.figure_handle;

%% Create the desired Laplacian

A = [0 1 0 0 0;
     1 0 1 1 0;
     0 1 0 1 1;
     0 1 1 0 1;
     0 0 1 1 2];

D = [0 0 0 0 0;
     0 3 0 0 0
     0 0 3 0 0;
     0 0 0 3 0;
     0 0 0 0 2];

L = D - A;


%Graph laplacian
%followers = -completeGL(N-1);
%L = zeros(N, N);
%L(2:N, 2:N) = followers;
%L(2, 2) = L(2, 2) + 1;
%L(2, 1) = -1;

%L(4, 2) = -1; % robot 4 follows robot 2
%L(4, 3) = -1; % robot 4 follows robot 3Maintain connexion from r4 to two robots
%L(2, 2) = L(2, 2) + 1; % Maintain connexion from r2 to one robot
%L(3, 3) = L(3, 3) + 1; % Maintain connexion from r3 to one robot
%L(4, 4) = L(4, 4) + 2; % Maintain connexion from r4 to two robots

%Initialize velocity vector
dxi = zeros(2, N);

%State for leader
state = 1;

% These are gains for our formation control algorithm
formation_control_gain = 10;
desired_distance = 0.4;

%% Grab tools we need to convert from single-integrator to unicycle dynamics

% Single-integrator -> unicycle dynamics mapping
si_to_uni_dyn = create_si_to_uni_dynamics('LinearVelocityGain', 0.8);
% Single-integrator barrier certificates
uni_barrier_cert = create_uni_barrier_certificate_with_boundary();
% Single-integrator position controller
leader_controller = create_si_position_controller('XVelocityGain', 0.8, 'YVelocityGain', 0.8, 'VelocityMagnitudeLimit', 0.1);

waypoints = [-1 0.6; -1 -0.6; 1 -0.6; 1 0.6]';

% Courbe de bézier ! (une sorte de milieu qui ne passe pas par tous les
% points)
t_bz = linspace(0, 1, 1000);
B = computeBezier(waypoints, t_bz);

% Plot the Bézier curve
%plot(B(1, :), B(2, :), 'm--', 'LineWidth', 2);
%legend('Waypoints', 'Graph Connections', 'Bézier Curve');

figure(fig);

% une spline !
waypoints = [waypoints, waypoints(:, 1)]; %boucle
t_points = 1:size(waypoints, 2); % Points de contrôle originaux (waypoints)
t_spline = linspace(1, size(waypoints, 2), 1000); % Points pour échantillonner la spline
spline_curve = spline(t_points, waypoints, t_spline); % Calcul de la courbe spline

% Tracé de la courbe spline
plot(spline_curve(1, :), spline_curve(2, :), 'm--', 'LineWidth', 2);
legend('Waypoints', 'Graph Connections', 'Spline Curve');


close_enough = 0.1;

%% Plotting Setup

% Color Vector for Plotting
% Note the Robotarium MATLAB instance runs in a docker container which will 
% produce the same rng value every time unless seeded by the user.
CM = ['k','b','r','g'];

%Marker, font, and line sizes
marker_size_goal = determine_marker_size(r, 0.20);
font_size = determine_font_size(r, 0.05);
line_width = 5;

% Create goal text and markers.
for i = 1:(size(waypoints, 2) - 1)
    % Text with goal identification
    goal_caption = sprintf('G%d', i);
    % Plot colored square for goal location.
    g(i) = plot(waypoints(1,i), waypoints(2,i),'s','MarkerSize',marker_size_goal,'LineWidth',line_width,'Color',CM(i));
    % Plot the goal identification text inside the goal location
    goal_labels{i} = text(waypoints(1,i)-0.05, waypoints(2,i), goal_caption, 'FontSize', font_size, 'FontWeight', 'bold');
end

% Plot graph connections
%Need location of robots
x=r.get_poses();

% Follower connections to each other
[rows, cols] = find(L == -1);

%Only considering half due to symmetric nature
for k = 1:length(rows)
    if rows(k) > cols(k)-1 % Éviter de tracer les doublons
        lf(k) = line([x(1,rows(k)), x(1,cols(k))], ...
                     [x(2,rows(k)), x(2,cols(k))], ...
                     'LineWidth', line_width, 'Color', 'b');
    end
end

% Leader connection assuming only connection between first and second
% robot.
ll = line([x(1,1), x(1,2)],[x(2,1), x(2,2)], 'LineWidth', line_width, 'Color', 'r'); 

% Follower plot setup
for j = 1:N-1    
    % Text for robot identification
    follower_caption{j} = sprintf('Follower Robot %d', j);
    % Plot the robot label text 
    follower_labels{j} = text(500, 500, follower_caption{j}, 'FontSize', font_size, 'FontWeight', 'bold');
end

%Leader plot setup
leader_label = text(500, 500, 'Leader Robot', 'FontSize', font_size, 'FontWeight', 'bold', 'Color', 'r');

%% Data Saving Setup

%Preallocate what we can.
robot_distance = zeros(5,iterations); % 4 distances and time
goal_distance = []; % Cannot preallocate this as we do not know how many
                   % times the goal will be reached.
start_time = tic;

r.step();
%index_target = 1;

%% CALCUL PRECISION

% Define pairs for distance error calculation using the adjacency matrix A
[distance_pairs_i, distance_pairs_j] = find(triu(A == 1));  % Upper triangle to avoid duplicates

% Define triplets for angle error calculation (you can adjust these based on your formation)
triplets = [1 2 3; 2 3 4; 3 4 5];  % Each row is a triplet of robot indices
desired_angles = [pi/2; pi/2; pi/2];  % Desired angles in radians for each triplet

% Initialize arrays to store errors over time
E_distance_array = zeros(1, iterations);
E_angle_array = zeros(1, iterations);




% Compute initial target index based on the leader's initial position
current_position = x(1:2, 1);  % Leader's initial position
% Compute distances to all points in spline_curve
distances = sqrt(sum((spline_curve - current_position).^2, 1));
% Find the index of the closest point
[~, index_target] = min(distances);

for t = 1:iterations

    figure(fig);
    %% Compute Errors

    % Compute distance error
    E_distance = 0;
    for k = 1:length(distance_pairs_i)
        i = distance_pairs_i(k);
        j = distance_pairs_j(k);
        d_ij = norm(x(1:2, i) - x(1:2, j));
        E_distance = E_distance + (d_ij - desired_distance)^2;
    end
    E_distance_array(t) = E_distance;

    % Compute angle error
    E_angle = 0;
    for k = 1:size(triplets, 1)
        i = triplets(k, 1);
        j = triplets(k, 2);
        k_idx = triplets(k, 3);
        v1 = x(1:2, i) - x(1:2, j);
        v2 = x(1:2, k_idx) - x(1:2, j);
        % Ensure vectors are not zero to avoid division by zero
        if norm(v1) > 0 && norm(v2) > 0
            cos_theta = dot(v1, v2) / (norm(v1) * norm(v2));
            % Clamp cos_theta to [-1, 1] to avoid numerical errors
            cos_theta = max(min(cos_theta, 1), -1);
            theta = acos(cos_theta);
            E_angle = E_angle + (theta - desired_angles(k))^2;
        end
    end
    E_angle_array(t) = E_angle;
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    
    %% Algorithm
    
    for i = 2:N
        
        %Zero velocity and get the topological neighbors of agent i
        dxi(:, i) = [0 ; 0];
        
        neighbors = topological_neighbors(L, i);
        
        for j = neighbors
            dxi(:, i) = dxi(:, i) + ...
                formation_control_gain*(norm(x(1:2, j) - x(1:2, i))^2 -  desired_distance^2)*(x(1:2, j) - x(1:2, i));
        end
    end
    
    %% Make the leader travel between waypoints
    
    current_position = x(1:2, 1);

    % Set the target position to the current index on the spline
    target_position = spline_curve(:, index_target);
    
    % Move towards the target
    dxi(:, 1) = leader_controller(current_position, target_position);
    
    % If the leader is close to the target position, move to the next point
    if norm(current_position - target_position) < close_enough  % target offset
        if index_target < length(t_spline)
            index_target = index_target + 1;  % Go to the next point
        else
            index_target = 1;  % Wrap around to the start of the spline
        end
    end
    
    % set the next discretized point on the curve as target
    target_position = spline_curve(:, index_target);
    
    % move towards the target
    dxi(:, 1) = leader_controller(current_position, target_position);

    %% Avoid actuator errors
    
    % To avoid errors, we need to threshold dxi
    norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
    threshold = 3/4*r.max_linear_velocity;
    to_thresh = norms > threshold;
    dxi(:, to_thresh) = threshold*dxi(:, to_thresh)./norms(to_thresh);
    
    %% Use barrier certificate and convert to unicycle dynamics
    dxu = si_to_uni_dyn(dxi, x);
    dxu = uni_barrier_cert(dxu, x);
    
    %% Send velocities to agents
    
    %Set velocities
    r.set_velocities(1:N, dxu);
    
    %% Update Plot Handles
    
    %Update position of labels for followers
    for q = 1:N-1
        follower_labels{q}.Position = x(1:2, q+1) + [-0.15;0.15];    
    end
    
    %Update position of graph connection lines
    for m = 1:size(lf, 2)
        % If not a graphics
        if ~isgraphics(lf(m))
            continue;
        end
        lf(m).XData = [x(1,rows(m)), x(1,cols(m))];
        lf(m).YData = [x(2,rows(m)), x(2,cols(m))];
    end
    
    %Update position of label and graph connection for leader
    leader_label.Position = x(1:2, 1) + [-0.15;0.15];
    ll.XData = [x(1,1), x(1,2)];
    ll.YData = [x(2,1), x(2,2)];
    
    % Resize Marker Sizes (In case user changes simulated figure window
    % size, this is unnecessary in submission as the figure window 
    % does not change size).

    marker_size_goal = num2cell(ones(1,length(waypoints))*determine_marker_size(r, 0.20));
    [g.MarkerSize] = marker_size_goal{:};
    font_size = determine_font_size(r, 0.05);
    leader_label.FontSize = font_size;
    
    for n = 1:N
        % Have to update font in loop for some conversion reasons.
        % Again this is unnecessary when submitting as the figure
        % window does not change size when deployed on the Robotarium.
        follower_labels{n}.FontSize = font_size;
        goal_labels{n}.FontSize = font_size;
    end
    
    %% Compute data to be saved and store in matrix.
    % Distances between connected robots.
    robot_distance(1,t) = norm([x(1:2,1) - x(1:2,2)],2);
    robot_distance(5,t) = toc(start_time);
    for b = 1:length(rows)/2+1
        robot_distance(b+1,t) = norm([x(1:2,rows(b)) - x(1:2,cols(b))],2);   
    end
    
    if(norm(x(1:2, 1) - waypoints) < close_enough)
        goal_distance = [goal_distance [norm(x(1:2, 1) - waypoints);toc(start_time)]];
    end
    
    %Iterate experiment
    r.step();
    %hide legend
    legend('off');
    Attenuations = net_utils.networkStep(x, 0); % No obstacles for now
end

% Save the data
save('DistanceData.mat', 'robot_distance');
save('GoalData.mat', 'goal_distance');

% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
r.debug();

%% Helper Functions

% Marker Size Helper Function to scale size with figure window
% Input: robotarium instance, desired size of the marker in meters
function marker_size = determine_marker_size(robotarium_instance, marker_size_meters)

% Get the size of the robotarium figure window in pixels
curunits = get(robotarium_instance.figure_handle, 'Units');
set(robotarium_instance.figure_handle, 'Units', 'Points');
cursize = get(robotarium_instance.figure_handle, 'Position');
set(robotarium_instance.figure_handle, 'Units', curunits);

% Determine the ratio of the robot size to the x-axis (the axis are
% normalized so you could do this with y and figure height as well).
marker_ratio = (marker_size_meters)/(robotarium_instance.boundaries(2) -...
    robotarium_instance.boundaries(1));

% Determine the marker size in points so it fits the window. cursize(3) is
% the width of the figure window in pixels. (the axis are
% normalized so you could do this with y and figure height as well).
marker_size = cursize(3) * marker_ratio;

end

% Font Size Helper Function to scale size with figure window
% Input: robotarium instance, desired height of the font in meters
function font_size = determine_font_size(robotarium_instance, font_height_meters)

% Get the size of the robotarium figure window in point units
curunits = get(robotarium_instance.figure_handle, 'Units');
set(robotarium_instance.figure_handle, 'Units', 'Points');
cursize = get(robotarium_instance.figure_handle, 'Position');
set(robotarium_instance.figure_handle, 'Units', curunits);

% Determine the ratio of the font height to the y-axis
font_ratio = (font_height_meters)/(robotarium_instance.boundaries(4) -...
    robotarium_instance.boundaries(3));

% Determine the font size in points so it fits the window. cursize(4) is
% the hight of the figure window in points.
font_size = cursize(4) * font_ratio;

end

% Save the error data
save('DistanceErrorData.mat', 'E_distance_array');
save('AngleErrorData.mat', 'E_angle_array');

% Optionally, plot the errors over time
fig2 = figure;
subplot(2,1,1);
plot(1:iterations, E_distance_array, 'LineWidth', 2);
xlabel('Iteration');
ylabel('Distance Error');
title('Distance Error over Time');

subplot(2,1,2);
plot(1:iterations, E_angle_array, 'LineWidth', 2);
xlabel('Iteration');
ylabel('Angle Error');
title('Angle Error over Time');
function B = computeBezier(P, t)
    n = size(P, 2) - 1;
    B = zeros(2, length(t));
    for k = 1:length(t)
        B(:, k) = [0; 0];
        for i = 0:n
            B(:, k) = B(:, k) + nchoosek(n, i) * (1 - t(k)).^(n - i) * t(k).^i * P(:, i + 1);
        end
    end
end