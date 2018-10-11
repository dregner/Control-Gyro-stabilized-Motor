%% Questao #2
clear;
clc;

% Definicoes
m = 0.5;
M = 1;
L = 1;
g = 9.81;

aux1 = -m*g/M;
aux2 = (m+M)*g/(M*L);

aux3 = 1/M;
aux4 = -1/(M*L);

A = [0 1  0   0;
     0 0 aux1 0;
     0 0  0   1;
     0 0 aux2 0]
 
B = [  0 ;
     aux3;
       0 ;
     aux4]

C = [1 0 0 0];

D = zeros(1,1);
% Modelo valido para theta aprox. 0 e theta_ponto aprox. 0

%% 2.1: Determine estabilidade interna da origem
disp('------Item 1------')

% Estabilidade interna ? verificada
% Para isso, todos os autovalores de A, que s?o polos de G(s)
% devem estar no SPE.
eig(A)
%Pergunta: E se tiver autovalor em 0?

%% 2.2: Verificar Controlabilidade e Observabilidade
disp('------Item 2------')

% Verificar controlabilidade

