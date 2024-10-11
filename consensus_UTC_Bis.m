%% Alexis LEBEL, Justin FERDINAND

%% Get Robotarium object used to communicate with the robots/simulator
N = 8;
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

[si_to_uni_dyn, uni_to_si_states] = create_si_to_uni_mapping();
uni_barrier_certificate = create_uni_barrier_certificate();

B = 0*[1;1;1;1;1;1;1;1];
Te = 0.1;

% Matrixes
L1 = 3*eye(N) - (ones(N)-eye(N)); 
% L2 = eye(N) - [0 0 0 1;
%                1 0 0 0;
%                0 1 0 0;
%                0 0 1 0];
% Select the number of iterations for the experiment.  This value is
% arbitrary
iterations = 5000;

% Degree - Adjacency

L2_anti_trigo = eye(N) - [
    0 1 0 0 0 0 0 0;  % Sommet 1 pointe vers 2
    0 0 1 0 0 0 0 0;  % Sommet 2 pointe vers 3
    0 0 0 1 0 0 0 0;  % Sommet 3 pointe vers 8
    0 0 0 0 1 0 0 0;  % Sommet 4 pointe vers 5
    0 0 0 0 0 1 0 0;  % Sommet 5 pointe vers 6
    0 0 0 0 0 0 1 0;  % Sommet 6 pointe vers 7
    0 0 0 0 0 0 0 1;  % Sommet 7 pointe vers 8
    1 0 0 0 0 0 0 0;  % Sommet 8 pointe vers 1
];
L2_trigo = eye(N) - [
    0 0 0 0 0 0 0 1;  % Sommet 1 pointe vers 2
    1 0 0 0 0 0 0 0;  % Sommet 2 pointe vers 3
    0 1 0 0 0 0 0 0;  % Sommet 3 pointe vers 8
    0 0 1 0 0 0 0 0;  % Sommet 4 pointe vers 5
    0 0 0 1 0 0 0 0;  % Sommet 5 pointe vers 6
    0 0 0 0 1 0 0 0;  % Sommet 6 pointe vers 7
    0 0 0 0 0 1 0 0;  % Sommet 7 pointe vers 8
    0 0 0 0 0 0 1 0;  % Sommet 8 pointe vers 1
];
K = -2; % Gain de Convergence
A = K * L2_anti_trigo;
[Ae, Be] = c2d(A, B, Te);

Xt = zeros(8, iterations);
Yt = zeros(8, iterations);
Temps = zeros(1, iterations);

% Iterate for the previously specified number of iterations
for t = 1:iterations
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();

    Xt(:, t) = x(1, :);
    Yt(:, t) = x(2, :);
    Temps(:, t) = t*Te;

    % if mod(t, 100) == 0
    %     if A == K * L2_anti_trigo
    %         A = -K * L2_trigo;
    %     else
    %         A = K * L2_anti_trigo;
    %     end
    %     [Ae, Be] = c2d(A, B, Te);
    % end

    % Ae*X va donner la prochaine valeur de X, donc pour avoir dxu, il faut
    % soustraire X de Ae*X
    dxi = [Ae*Xt(:, t) - Xt(:, t), Ae*Yt(:, t) - Yt(:, t)]';
    % dxu = algorithm(x);

    dxu = si_to_uni_dyn(dxi, x);
    
    %% Send velocities to agents

    % dxu = uni_barrier_certificate(dxu, x);
    
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dxu);
    
    % Send the previously set velocities to the agents.  This function must be called!
    r.step();
end

for i = 1:N
    plot(Xt(i,:), Yt(i,:));
end