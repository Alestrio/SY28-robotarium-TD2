% Same script as leader_follower_with_plotting.m but with additional data saving. Two
% data sets will be saved, one saving the distance between connected robots
% through time, and another with the distance between the leader and goal
% location when the goal is "reached".

% Sean Wilson
% 07/2019

close all;


%% Experiment Constants

%Run the simulation for a specific number of iterations
iterations = 5000;

%% Set up the Robotarium object

N = 5;
initial_positions = generate_initial_conditions(N, 'Width', 1, 'Height', 1, 'Spacing', 0.3);
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);
net_utils = SY28_network_utils();
% Définir les obstacles dans l'environnement (par exemple des cercles de rayon R)
obstacles = [0.5, 0.5; -0.5, -0.2; 0.0, -0.8]; % Liste des positions des obstacles (x, y)
antenna = [0.6, 0.65]; % Position de l'antenne
fig = r.figure_handle;

%% Create the matrix Laplacian

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

L_diamond = D - A;

L = L_diamond;

L_line = [1 -1 0 0 0;
          -1 2 -1 0 0;
          0 -1 2 -1 0;
          0 0 -1 2 -1;
          0 0 0 -1 1];

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
desired_distance = 0.1;

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

figure(fig);
% Plot the Bézier curve
%plot(B(1, :), B(2, :), 'm--', 'LineWidth', 2);
%legend('Waypoints', 'Graph Connections', 'Bézier Curve');

% une spline  (comme la courbe mais qui passe par tous les points)
waypoints = [waypoints, waypoints(:, 1)]; %boucle
t_points = 1:size(waypoints, 2); % Points de contrôle originaux (waypoints)
t_spline = linspace(1, size(waypoints, 2), 1000); % Points pour échantillonner la spline
spline_curve = spline(t_points, waypoints, t_spline); % Calcul de la courbe spline

% Tracé de la courbe spline
plot(spline_curve(1, :), spline_curve(2, :), 'm--', 'LineWidth', 2);
legend('Waypoints', 'Graph Connections', 'Spline Curve');

%offset (assez réduit ici)
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
%% OBSTACLES
% Définir les obstacles dans l'environnement (par exemple des cercles de rayon R)
obstacles = [0.5, 0.5; -0.5, -0.2; 0.0, -0.7]; % Liste des positions des obstacles (x, y)
obstacle_radius = 0.15; % Rayon de chaque obstacle
% Tracer les obstacles sur la figure
hold on;
num_obstacles = size(obstacles, 1);
for i = 1:num_obstacles
    theta = linspace(0, 2*pi, 50); % Pour dessiner un cercle
    x_circle = obstacle_radius * cos(theta) + obstacles(i, 1);
    y_circle = obstacle_radius * sin(theta) + obstacles(i, 2);
    fill(x_circle, y_circle, 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'r'); % Tracé en rouge avec transparence
end
% plot the antenna
plot(antenna(1), antenna(2), 'kx', 'MarkerSize', 10, 'LineWidth', 2);
hold off;

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

% Initialize arrays to store errors over time
E_distance_array = zeros(1, iterations);


%% Compute nearest point on the curve for the leader

% Compute initial target index based on the leader's initial position
current_position = x(1:2, 1);  % Leader's initial position
% Compute distances to all points in spline_curve
distances = sqrt(sum((spline_curve - current_position).^2, 1));
% Find the index of the closest point
[~, index_target] = min(distances);
target_position = spline_curve(:, index_target);

% variable for smoother pathing
on_spline = false; %flag to know if leader is on spline or not
dx_dt = diff(spline_curve(1, :)) ./ diff(t_spline);
dy_dt = diff(spline_curve(2, :)) ./ diff(t_spline);

 % flag for formation changes
in_line_formation = false;

%% MAIN LOOP
for t = 1:iterations
    figure(fig);
    % disable legend
    legend('off');
    
    %% Formation changes
    line_threshold     = 0.1; 
    diamond_threshold  = 0.15;

    min_dist = inf; %reset

    % calculate min_dist.
    for i = 1:N
        robot_pos = x(1:2, i);  % define robot_pos!
        for obs_idx = 1:num_obstacles
            obstacle_pos = obstacles(obs_idx, :)';  % define obstacle_pos!
            dist_to_obstacle = norm(robot_pos - obstacle_pos);
    
            if dist_to_obstacle < min_dist
                min_dist = dist_to_obstacle; 
            end
        end
    end
    
    if min_dist < line_threshold
        in_line_formation = true;
    end
    
    % hysteris snippet
    if in_line_formation
        % If we are currently in line formation,
        % revert to diamond if we are safely away:
        if min_dist > diamond_threshold
            in_line_formation = false;
            L = L_diamond;
        else
            L = L_line;   % remain in line
        end
    else
        % If currently in diamond,
        % switch to line if too close to any obstacle:
        if min_dist < line_threshold
            in_line_formation = true;
            L = L_line;
        else
            L = L_diamond;  % remain in diamond
        end
    end

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
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    
    %% Path following algorithm
    
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
    
    if index_target < length(t_spline)
        dx = dx_dt(index_target);
        dy = dy_dt(index_target);
    else
        % If we're at the end, use the previous point’s derivative
        dx = dx_dt(index_target-1);
        dy = dy_dt(index_target-1);
    end

    tangent = [dx; dy];
    tangent_norm = norm(tangent);
    if tangent_norm > 0
        tangent = tangent / tangent_norm;
    else
        % If derivative is zero (should be rare), fallback to direct approach
        tangent = [1; 0]; 
    end

    look_ahead_distance = 1; % Adjust this parameter for smoother or sharper turns
    smooth_target = target_position + look_ahead_distance * tangent;


    %% Move towards the target
    
    %smooth way (disabled due to a bug)
    % dxi(:, 1) = leader_controller(current_position, smooth_target);

    % harsh way (enabled)
    dxi(:, 1) = leader_controller(current_position, target_position); 

    % If the leader is close to the target position, move to the next point
    if norm(current_position - target_position) < close_enough  % target offset
        if index_target < length(t_spline)
            index_target = index_target + 1;  % Go to the next point
        else
            index_target = 1;  % Wrap around to the start of the spline
        end
    end


    %% Obstacle avoidance algorithm
    repulsion_gain = 2.0;      % Strong push if too close
    tangent_gain   = 0.5;      % Weaker push when we're just in the "avoid" zone
    radial_gain    = 0.2;      % Slight outward push in tangent zone
    critical_dist  = obstacle_radius + 0.01;  % Start avoiding sooner
    
    for i = 1:N
        robot_pos = x(1:2, i); 
        for obs_idx = 1:size(obstacles, 1)
    
            obstacle_position = obstacles(obs_idx, :)';
            dist_to_obstacle  = norm(robot_pos - obstacle_position);
    
            if dist_to_obstacle < obstacle_radius
                % ----------------------------------------------------------
                % INSIDE the obstacle zone => big repulsion outward
                % ----------------------------------------------------------
                direction_out = (robot_pos - obstacle_position) / dist_to_obstacle;
                dxi(:, i) = dxi(:, i) + repulsion_gain * direction_out;
    
            elseif dist_to_obstacle < critical_dist
                % ----------------------------------------------------------
                % TANGENT avoidance zone => gently steer around
                % ----------------------------------------------------------
                direction_out = (robot_pos - obstacle_position) / dist_to_obstacle;
                tangent_dir   = [-direction_out(2); direction_out(1)];
    
                dxi(:, i) = dxi(:, i) ...
                            + tangent_gain * tangent_dir ...
                            + radial_gain  * direction_out;
            end
        end
    end




    %% Avoid actuator errors
    
    % To avoid errors, we need to threshold dxi
    norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
    threshold = 3/4*r.max_linear_velocity;
    to_thresh = norms > threshold;
    dxi(:, to_thresh) = threshold*dxi(:, to_thresh)./norms(to_thresh);
    
    %% Use barrier certificate and convert to unicycle dynamics
    dxu = si_to_uni_dyn(dxi, x);
    dxu = uni_barrier_cert(dxu, x);
    dxu = si_to_uni_dyn(dxi, x);
    dxu = uni_barrier_cert(dxu, x);
    
    %% Send velocities to agents
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
    [Attenuations, BERs, ReceivedPowers] = net_utils.networkStep([x , [antenna(1) ; antenna(2) ; 0]], obstacles);

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

% Optionally, plot the errors over time
figure;
subplot(2,1,1);
plot(1:iterations, E_distance_array, 'LineWidth', 2);
xlabel('Iteration');
ylabel('Distance Error');
title('Distance Error over Time');

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