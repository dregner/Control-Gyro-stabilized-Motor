% Pendulo Invertido linear
clc
clear all;
close all;
%% Variaveis do sistema
M = 1;
m = 0.5;
L = 1;
g = 9.81;

%% matrizes do sistema

A = [0 1 0 0; 
    0 0 -m*g/M 0;
    0 0 0 1;
    0 0 (m+M)*g/(M*L) 0];
C = [1 0 0 0]; % saida questao 2
B = [0; 1/M; 0; -1/(M*L)];

Dsim = zeros(4,1);
x0 = [0 0 pi/180*10 0]; % condicao inicial do sistema
x0obs = [0 0 pi/180*10 0]; % condicao inicial do observador


%% Controlabilidade e Observabilidade Letra B

O = [C; C*A; C*A^2; C*A^3];
Cont = [B A*B A^2*B A^3*B];
    
%% ganho K

T = [A^3*B-14.715*A*B, A^2*B-14.715*B, A*B, B];
d = [5.0625, 13.5, 13.5, 6];
a = [0 , 0, -14.715, 0];
kd = d-a;
Kl = kd*inv(T);
eig(A-B*Kl)

%% calculo de L

Ab = transpose(A);
Bb = transpose(C);
Ob = [Bb Ab*Bb Ab^2*Bb Ab^3*Bb];

Tb = [Ab^3*Bb-14.715*Ab*Bb, Ab^2*Bb-14.715*Bb, Ab*Bb, Bb];
db = [256 256 96 16];
ab = a;
kdb = db-ab;
Kb = kdb*inv(Tb)
L = Kb'
eig(A-L*C)
%% Modelo Interno Seguir degrau e rejeita pert. degrau. Letra C
coef = [1 0];
M = [0];
N = [1];
Am = M;
Bm = N;
Cm = eye(1);
Dm = zeros(1,1);

Aa = [A zeros(4,1); -Bm*C Am];
Ba = [B; zeros(1,1)];
Ca = [C 0];

pd = -2;
Ka = place(Aa,Ba,[pd pd-0.025 pd-0.05 pd-0.075 pd-0.1]);
r = 0.001;
R = r;
Q = eye(5);
Ka = lqr(Aa,Ba,Q,R);

% Verificacao
polosMF = eig(Aa-Ba*Ka)

% Matriz de ganho para o estado da planta x 
K = Ka(:,1:4);

% Matriz de ganho para o estado do modelo interno xm
Km = Ka(:,5);

%% Funcao transferencia malha fechada 2.3
s = tf('s');

Ae = Aa-Ba*Ka;
Be = [B zeros(4,1);zeros(1,1) Bm];
Ga = Ca*(s*eye(5)-Ae)^-1*Be;
Ga = minreal(zpk(Ga))
figure
pzmap(Ga);
figure
step(Ga);


%%
D = [0]
sys=ss(A,B,C,D);   % define modelo de estado
 
G=zpk(tf(sys))     % matriz de transferencia

%% Modelo Interno  Seguir degrau, rejeita degra e seno 0.1 rad/s Letra E

coef = [0 -.1^2 0]
M = [zeros(2,1) eye(2); coef];
N = [0;0;1];
Am = M;
Bm = N;
Cm = eye(3);
Dm = zeros(3,1);


Aa = [A zeros(4,3); -Bm*C Am];
Ba = [B; zeros(3,1)];
Ca = [C 0 0 0];

pd = -1.5;
Ka = place(Aa,Ba,[pd pd-0.025 pd-0.05 pd-0.075 pd-0.1 pd-0.125 pd-0.15]);
%%
r = 1;
R = r;
Q = eye(7);
Ka = lqr(Aa,Ba,Q,R);
% Verificacao
polosMF = eig(Aa-Ba*Ka)

% Matriz de ganho para o estado da planta x 
K = Ka(:,1:4);

% Matriz de ganho para o estado do modelo interno xm
Km = Ka(:,5:7);

%% Função de Transferencia Malha fechada 2.5

Ae = Aa-Ba*Ka;
Be = [B zeros(4,1);zeros(3,1) Bm];
Ga = Ca*(s*eye(7)-Ae)^-1*Be;
Ga = minreal(zpk(Ga))
figure
pzmap(Ga);
figure
step(Ga);

%% Observador

% Polo repetido desejado para o observador
pobs = -8;
% Matriz de ganho para posicionar os polos de A-LC em pobs
H = place(A',C',[pobs pobs-0.025 pobs-0.05 pobs-0.075]);
L = H';

V1 =0.001*eye(4);
V2 = 0.0005;
q = 1;
L = lqr(A',C',V1,q*V2)';

polosObs = eig(A-L*C)

%% PLOTS
%close all
model = 'modelos';
load_system(model);
figure(1)
plot(tout,x(:,2),tout,x(:,3),tout,x(:,4), tout, x(:,5)) % plot estados
grid
title('Realimentação Estados')
figure(2)
plot(tout, y(:,2),tout,y(:,3))
grid
title('Realimentação de Estados')
% figure(3)
% plot(tout,x1(:,2),'r-',tout,x1(:,3),'b-',tout,x1(:,4),'g-', tout, x1(:,5),'y-') % plot estados
% hold on
% plot(tout,x2(:,2),'r--',tout,x2(:,3),'b--',tout,x2(:,4),'g--', tout, x2(:,5),'y--') % plot estados
% hold off
% grid
% title('Controlador Observador')
% figure(4)
% plot(tout(1,, y1(:,2),tout,y1(:,3))
% grid
% title('Controlador-Observador');

%%
d = [128 448 672 560 280 84 14]
p = [0 0 0 -2943/20000 0 -2941/200 0]
v7 = Ba
v6 = Aa*v7+p(7)*Ba
v5 = Aa*v6+p(6)*Ba
v4 = Aa*v5+p(5)*Ba
v3 = Aa*v4+p(4)*Ba
v2 = Aa*v3+p(3)*Ba
v1 = Aa*v2+p(2)*Ba
T = [v1 v2 v3 v4 v5 v6 v7]
kb = d-p
K = kb*T^-1
eig(Aa-Ba*K)

%%
syms x1 x2 u
q1 = -sin(x1)+2*x2+5*x1*x2^2;
q2 = 10*x1^5+2*exp(x2)-2+3*cos(x1)*u;
q = [q1;q2];
x = [x1 x2]
O = (jacobian(q,x))
