clc
clear all
close all

%% Questao 3

M = 1;
L = 1;
m = 0.5;
g = 9.81;

syms x1 x2 x3 x4 u;
x = [x1 x2 x3 x4];
C0 = 0;
x0 = [0 0 pi/180*0 0];
x0obs = [0; 0; pi/180*0; 0];
%% Equações não lineares para cada estado
% dx1 = f1, dx2 = f2, dx3 = f3, dx4 = f4
f1 = x2;
f2 = (u-m*g*sin(x3)*cos(x3)+m*L*x4^2*sin(x3))/(M+m-m*cos(x3)^2);
f3 = x4;
f4_num = m*g*sin(x3)-((u+m*L*x4^2*sin(x3))*m*cos(x3))/(M+m);
f4_den = m*L-(m^2*L*cos(x3)^2)/(M+m);
f4 = f4_num/f4_den;
f = [f1;f2;f3;f4];

A = double(subs(jacobian(f,x),[x1 x2 x3 x4],[0 0 0 0])); %nao entendo pq retorna matriz 4x3
B = double(subs(jacobian(f,u),[x3 x4 u],[0 0 0]));
C = [1 0 0 0];
D = zeros(2,2);
Con = ctrb(A,B); %Matriz Controlabilidade
rank(Con);


pd = -5;
K = place(A,B, [pd pd-.01 pd+0.05 pd+0.02]);

%% Sistema Aumentado para controle integral linear
Aa=[ A zeros(4,1);
    -C 0];

Ba=[B ; 0];

% Verifica a controlabilidade do Sistema Aumentado
Con = ctrb(Aa,Ba);
vsCon = svd(Con);

% Verifica a observabilidade da Planta
Obs = obsv(A,C);
vsObs = svd(Obs);

pd = -4;
Kc = place(Aa, Ba, [pd pd-0.01 pd-0.02 pd-0.05 pd+0.01]);
Kx = Kc(1,1:4);
Km = Kc(1,5);
%% Observador

pd = -10;
L=place(A',C',[pd pd-0.05 pd-0.03 pd-0.04])';        % polo duplo de A-LC em s=-12     
rl =0.01;
V1  =0.001*eye(4);
V2 = 0.0005;
Lk = lqr(A', C', V1,rl*V2);
Lk = Lk'
eig(A-Lk*C)
%% LQR
x0 = [0 0 pi/180*20 0];
x0obs = [0; 0; pi/180*20; 0];
Q = eye(5);
r = 1;
R = eye(1);
Ka = lqr(Aa,Ba,Q,r*R);
Kx = Ka(1,1:4)
Km = Ka(1,5)
eig(Aa-Ba*Ka)

pd = -5;
L=place(A',C',[pd pd-0.05 pd-0.03 pd-0.04])';   
%% PLOTS
%close all
    % model = 'modelo_q3';
% load_system(model);
%     figure(1)
% plot(tout,x(:,2),tout,x(:,3),tout,x(:,4), tout, x(:,5)) % plot estados
% grid
% legend('x1','x2','x3','x4');
% title('Realimentação Estados')
% figure(2)
% plot(tout, y(:,2),tout,y(:,3))
% legend('y','u');
% grid
% title('Realimentação de Estados')
figure(3)
plot(tout,x1(:,5),'r-',tout,x1(:,4),'b-',tout,x1(:,3),'g-', tout, x1(:,2),'y-') % plot estados
hold on
plot(tout,x2(:,2),'r--',tout,x2(:,3),'b--',tout,x2(:,4),'g--', tout, x2(:,5),'y--') % plot estados
hold off
grid
legend('x1','x2','x3','x4','x1e','x2e','x3e','x4e');
title('Controlador Observador')
figure(4)
plot(tout,y1(:,2),tout,y1(:,3))
grid
legend('y','u');
title('Controlador-Observador');