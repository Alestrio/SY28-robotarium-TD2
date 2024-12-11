%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% This script defines functions for the propagation models of the communications between the agents.  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% This class defines a utility to simulate the network of agents in the urban environment.            %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Author: Alexis LEBEL, Date: 2024-12-03                                                              %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef SY28_network_utils
    properties
        street_width = 20; % meters
        frequency = 1710; % MHz
        building_height = 20; % meters
        rx_height = 2; % meters, height of antenna on the car
        tx_height = 2; % meters, height of antenna on the car
        angle = 0; % degrees, we assume both antenna are omnidirectional
        in_between_building_distance = 10; % meters
        % Figure to be updated
        fig
    end


    
    methods
        function obj = SY28_network_utils(street_width, frequency, building_height, rx_height, tx_height, angle, in_between_building_distance)
            if nargin > 0
                obj.street_width = street_width;
                obj.frequency = frequency;
                obj.building_height = building_height;
                obj.rx_height = rx_height;
                obj.tx_height = tx_height;
                obj.angle = angle;
                obj.in_between_building_distance = in_between_building_distance;
            end
            obj.fig = figure;
            % x is time, y is attenuation
            xlabel('Time');
            ylabel('Attenuation (dB)');
            title('Attenuations between agents');
            % show the figure
        end
        
        function Attenuations = networkStep(obj, agents, environment)
            persistent steps;
            if isempty(steps)
                steps = 0;
            end
            steps = steps + 1;
            Attenuations = obj.computeAllAttenuations(agents, environment);
            obj.addNewAttenuationsToPlot(Attenuations, steps);
        end
    end
    
    methods (Access = private)
        function Attenuations = computeAllAttenuations(obj, agents, environment)
            numAgents = size(agents, 2); % Nombre d'agents
            Attenuations = zeros(numAgents, numAgents);
            for i = 1:numAgents
                for j = 1:numAgents
                    if i ~= j
                        % Extraire les coordonnées des agents
                        agent_i = struct('x', agents(1, i), 'y', agents(2, i), 'theta', agents(3, i));
                        agent_j = struct('x', agents(1, j), 'y', agents(2, j), 'theta', agents(3, j));
                        
                        % Calculer la distance entre les agents
                        distance = obj.computeDistance(agent_i, agent_j);
                        
                        % Vérifier la ligne de vue (LoS)
                        if obj.checkLoS(agent_i, agent_j, environment)
                            Attenuations(i, j) = PropagationModel.WalfishIkegami_LOS(distance, obj.frequency);
                        else
                            Attenuations(i, j) = PropagationModel.WalfishIkegami_NLOS(obj.street_width, obj.frequency, obj.building_height, obj.rx_height, obj.angle, obj.tx_height, distance, obj.in_between_building_distance);
                        end
                    end
                end
            end
        end
        
        function distance = computeDistance(~, agent1, agent2)
            % Agents are in 2D space, a struct with fields x, y, theta
            distance = sqrt((agent1.x - agent2.x)^2 + (agent1.y - agent2.y)^2)/100;
        end
        
        function LoS = checkLoS(~, agent1, agent2, obstacles)
            % agents are in 2D space, a vector 3 rows, x, y, theta
            % obstacles are in 2D space, a vector 3 rows, x, y, radius
            % True placeholder
            LoS = true; % Placeholder TODO: Implement actual LoS check
        end
        
        function addNewAttenuationsToPlot(obj, Attenuations, steps)
            % Attenuations is a matrix of size numAgents x numAgents
            % Symmetric matrix, so we only need to plot the upper triangle
            % this function will plot the attenuations y and time x
            
            % Define static variables for x and y buffers
            persistent xBuffer yBuffer;
            if isempty(xBuffer)
                xBuffer = [];
                yBuffer = cell(size(Attenuations));
            end
            
            % Get the number of agents
            numAgents = size(Attenuations, 1);
            
            % Get the current time
            currentTime = datetime('now');
            
            % Loop through the upper triangle of the matrix
            for i = 1:numAgents
                for j = i+1:numAgents
                    % Add the current time and attenuation value to the buffers
                    xBuffer = [xBuffer; currentTime];
                    yBuffer{i, j} = [yBuffer{i, j}; Attenuations(i, j)];
                end
            end
            
            
            
            %every 30 stepss
            if mod(steps, 30) == 0
                % Create a new figure
                figure(obj.fig);
                hold on;
                % Clear the plot
                cla;
                % Plot each attenuation time series
                for i = 1:numAgents
                    for j = i+1:numAgents
                        if ~isempty(yBuffer{i, j})
                            % create name
                            name = strcat('Agent ', num2str(i), ' to Agent ', num2str(j));
                            % Create a timeseries object
                            ts = timeseries(yBuffer{i, j}, 'Name', name);
                            
                            % Plot the timeseries
                            plot(ts, '-', 'DisplayName', name);
                        end
                    end
                end
                % Add labels and title
                xlabel('Time');
                ylabel('Attenuation (dB)');
                title('Attenuations Between Agents Over Time');
                legend('show');
                hold off;
            end
            
        end
    end
end