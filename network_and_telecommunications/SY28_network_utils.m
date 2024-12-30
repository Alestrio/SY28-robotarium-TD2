classdef SY28_network_utils
    properties
        street_width = 20; % meters
        frequency = 1710; % MHz
        building_height = 20; % meters
        rx_height = 2; % meters
        tx_height = 2; % meters
        antenna_tx_height = 10; % meters
        angle = 0; % degrees
        in_between_building_distance = 10; % meters
        emission_power_dbm = -15; % Transmission power in dBm
        noise_floor_dbm = -90; % Typical noise floor
        antenna_drawn = true;
        % Figures for each metric
        fig_attenuation
        fig_power
        fig_ber
    end
    
    methods
        function obj = SY28_network_utils(street_width, frequency, building_height, rx_height, tx_height, angle, in_between_building_distance, antenna_drawn)
            if nargin > 0
                obj.street_width = street_width;
                obj.frequency = frequency;
                obj.building_height = building_height;
                obj.rx_height = rx_height;
                obj.tx_height = tx_height;
                obj.angle = angle;
                obj.in_between_building_distance = in_between_building_distance;
                obj.antenna_drawn = antenna_drawn;
            end
            
            % Create separate figures for each metric
            obj.fig_attenuation = figure('Name', 'Attenuations');
            xlabel('Time');
            ylabel('Attenuation (dB)');
            title('Attenuations between agents');
            
            obj.fig_power = figure('Name', 'Received Powers');
            xlabel('Time');
            ylabel('Received Power (dBm)');
            title('Received Powers between agents');
            
            obj.fig_ber = figure('Name', 'Bit Error Rates');
            xlabel('Time');
            ylabel('BER');
            title('Bit Error Rates between agents');
            obj.antenna_drawn = true;
        end
        
        function [Attenuations, ReceivedPowers, BERs] = networkStep(obj, agents, environment)
            persistent steps;
            if isempty(steps)
                steps = 0;
            end
            steps = steps + 1;
            
            [Attenuations, ReceivedPowers, BERs] = obj.computeNetworkMetrics(agents, environment);
            obj.addNewMetricsToPlot(Attenuations, ReceivedPowers, BERs, steps);
        end
    end
    
    methods (Access = private)
        function [Attenuations, ReceivedPowers, BERs] = computeNetworkMetrics(obj, agents, environment)
            numAgents = size(agents, 2);
            Attenuations = zeros(numAgents, numAgents);
            ReceivedPowers = zeros(numAgents, numAgents);
            BERs = zeros(numAgents, numAgents);
            
            for i = 1:numAgents
                for j = i+1:numAgents  % Only compute for upper triangle
                    % Extract agent positions
                    agent_i = struct('x', agents(1, i), 'y', agents(2, i), 'theta', agents(3, i));
                    agent_j = struct('x', agents(1, j), 'y', agents(2, j), 'theta', agents(3, j));
                    
                    % Compute distance and check LoS
                    distance = obj.computeDistance(agent_i, agent_j);
                    hasLoS = obj.checkLoS(agent_i, agent_j, environment);
                    
                    % Calculate path loss
                    if hasLoS
                        Attenuations(i, j) = PropagationModel.WalfishIkegami_LOS(distance, obj.frequency);
                    elseif (i == numAgents || j == numAgents) % Antenna
                        Attenuations(i, j) = PropagationModel.WalfishIkegami_NLOS(...
                            obj.street_width, obj.frequency, obj.building_height, ...
                            obj.rx_height, obj.angle, obj.antenna_tx_height, distance, ...
                            obj.in_between_building_distance);
                    else
                        Attenuations(i, j) = PropagationModel.WalfishIkegami_NLOS(...
                            obj.street_width, obj.frequency, obj.building_height, ...
                            obj.rx_height, obj.angle, obj.tx_height, distance, ...
                            obj.in_between_building_distance);
                    end
                    
                    % Calculate received power
                    ReceivedPowers(i, j) = obj.emission_power_dbm - Attenuations(i, j);
                    
                    % Calculate SNR and BER
                    snr_db = ReceivedPowers(i, j) - obj.noise_floor_dbm;
                    snr_linear = 10^(snr_db/10);
                    BERs(i, j) = 0.5 * (1 - sqrt(snr_linear/(1 + snr_linear)));
                end
            end
        end
        
        function addNewMetricsToPlot(obj, Attenuations, ReceivedPowers, BERs, steps)
            % Define static variables for buffers
            persistent attBuffer powerBuffer berBuffer timeBuffer;
            if isempty(attBuffer)
                attBuffer = cell(size(Attenuations));
                powerBuffer = cell(size(ReceivedPowers));
                berBuffer = cell(size(BERs));
                timeBuffer = [];
            end
            
            % Add current time step to buffer
            timeBuffer = [timeBuffer; steps];
            
            % Get the number of agents
            numAgents = size(Attenuations, 1);
            
            % Update buffers
            for i = 1:numAgents
                for j = i+1:numAgents
                    attBuffer{i, j} = [attBuffer{i, j}; Attenuations(i, j)];
                    powerBuffer{i, j} = [powerBuffer{i, j}; ReceivedPowers(i, j)];
                    berBuffer{i, j} = [berBuffer{i, j}; BERs(i, j)];
                end
            end
            
            % Update plots every 50 steps
            if mod(steps, 50) == 0
                % Plot Attenuations
                figure(obj.fig_attenuation);
                hold on;
                cla;
                for i = 1:numAgents
                    for j = i+1:numAgents
                        if ~isempty(attBuffer{i, j})
                            name = sprintf('Agent %d to Agent %d', i, j);
                            plot(timeBuffer, attBuffer{i, j}, '-', 'DisplayName', name, 'LineWidth', 2);
                        end
                    end
                end
                xlabel('Time Steps');
                ylabel('Attenuation (dB)');
                title('Attenuations Between Agents');
                grid on;
                legend('show');
                hold off;
                
                % Plot Received Powers
                figure(obj.fig_power);
                hold on;
                cla;
                for i = 1:numAgents
                    for j = i+1:numAgents
                        if ~isempty(powerBuffer{i, j})
                            if obj.antenna_drawn && (i == numAgents || j == numAgents)
                                name = sprintf('Agent %d to Antenna', min(i, j));
                            else
                                name = sprintf('Agent %d to Agent %d', i, j);
                            end
                            plot(timeBuffer, powerBuffer{i, j}, '-', 'DisplayName', name, 'LineWidth', 2);
                        end
                    end
                end
                xlabel('Time Steps');
                ylabel('Received Power (dBm)');
                title('Received Powers Between Agents');
                grid on;
                legend('show');
                hold off;
                
                % Plot BERs
                figure(obj.fig_ber);
                hold on;
                cla;
                for i = 1:numAgents
                    for j = i+1:numAgents
                        if ~isempty(powerBuffer{i, j})
                            if obj.antenna_drawn && (i == numAgents || j == numAgents)
                                name = sprintf('Agent %d to Antenna', min(i, j));
                            else
                                name = sprintf('Agent %d to Agent %d', i, j);
                            end
                            plot(timeBuffer, berBuffer{i, j}, '-', 'DisplayName', name, 'LineWidth', 2);
                        end
                    end
                end
                xlabel('Time Steps');
                ylabel('Bit Error Rate');
                title('Bit Error Rates Between Agents');
                grid on;
                legend('show');
                hold off;
            end
        end
        
        function distance = computeDistance(~, agent1, agent2)
            distance = sqrt((agent1.x - agent2.x)^2 + (agent1.y - agent2.y)^2)/100;
        end
        
        function LoS = checkLoS(~, agent1, agent2, obstacles)
            % Check if there is a clear line of sight between two agents considering obstacles
            %
            % Inputs:
            %   agent1: struct with fields x, y representing the first agent's position
            %   agent2: struct with fields x, y representing the second agent's position
            %   obstacles: Nx2 array where each row represents [x, y] coordinates of obstacles
            %
            % Output:
            %   LoS: boolean, true if there is a clear line of sight, false otherwise
        
            % If no obstacles, there is always line of sight
            if isempty(obstacles)
                LoS = true;
                return;
            end
        
            % Get line segment endpoints
            x1 = agent1.x;
            y1 = agent1.y;
            x2 = agent2.x;
            y2 = agent2.y;
        
            % Consider each obstacle as having a small radius
            obstacle_radius = 0.150; % 150cm radius for obstacles
        
            % Check intersection with each obstacle
            for i = 1:size(obstacles, 1)
                % Get obstacle position
                obstacle_x = obstacles(i, 1);
                obstacle_y = obstacles(i, 2);
        
                % Calculate the minimum distance between the line segment and the obstacle
                % using point-to-line-segment distance formula
        
                % Vector from line start to end
                line_vec = [x2 - x1; y2 - y1];
                line_length = norm(line_vec);
                line_unit_vec = line_vec / line_length;
        
                % Vector from line start to obstacle
                point_vec = [obstacle_x - x1; obstacle_y - y1];
        
                % Project point_vec onto line_unit_vec
                projection_length = dot(point_vec, line_unit_vec);
        
                % If projection is outside line segment, use distance to endpoint
                if projection_length < 0
                    min_dist = norm([obstacle_x - x1; obstacle_y - y1]);
                elseif projection_length > line_length
                    min_dist = norm([obstacle_x - x2; obstacle_y - y2]);
                else
                    % Calculate perpendicular distance
                    projection = line_unit_vec * projection_length;
                    perpendicular = point_vec - projection;
                    min_dist = norm(perpendicular);
                end
        
                % If minimum distance is less than obstacle radius, line of sight is blocked
                if min_dist < obstacle_radius
                    LoS = false;
                    return;
                end
            end
        
            % If we've checked all obstacles and found no intersections, there is line of sight
            LoS = true;
        end
    end
end