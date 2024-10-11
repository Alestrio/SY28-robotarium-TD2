% Alexis LEBEL, Justin FERDINAND

clear
close all

N = 8;
B = 0*[1;1;1;1;1;1;1;1];
Te = 0.1;

% Matrixes
L1 = 3*eye(N) - (ones(N)-eye(N)); 
% L2 = eye(N) - [0 0 0 1;
%                1 0 0 0;
%                0 1 0 0;
%                0 0 1 0];
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
K = 0.4; % Gain de Convergence
A = K * L2_trigo;
[Ae, Be] = c2d(A, B, Te);

% Positions Sens anti-trigo
X=[-2 0 2 2 2   0 -2 -2]';
Y=[2  2 2 0 -2 -2 -2  0]'; 
% Positions Sens trigo
% X=[-2 -2 -2  0 2  2 2 0]';
% Y=[2   0 -2 -2 2  0 -2 2]'; 

% Vecteurs
M = 100;
Xt = zeros(8, M);
Yt = zeros(8, M);
Temps = zeros(1, M);

for k = 1:M
    Xt(:, k) = X;
    Yt(:, k) = Y;
    Temps(:, k) = k*Te;
    X = Ae*X;
    Y = Ae*Y;
    if mod(k, 15) == 0
        if A == K * L2_anti_trigo
            A = -K * L2_trigo;
        else
            A = K * L2_anti_trigo;
        end
        [Ae, Be] = c2d(A, B, Te);
    end
end

figure(1);
clf;
hold on;

for i = 1:N
    plot(Xt(i,:), Yt(i,:));
    pause(0.01);
end

% for i = 1:M
%     plot(Xt(:,i), Yt(:,i));
%     pause(0.001);
% end