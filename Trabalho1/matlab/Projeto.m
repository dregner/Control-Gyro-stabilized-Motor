clc
clear all
close all
%%
% Parâmetros do modelo
Mv = 0.152; % Massa do veículo sem giro [kg]
Mg = 0.15; % Massa do giro [kg]
Rg = 0.095/2; % Raio do giro [m]
Ag = 0.006; % Espessura giro [m]
Av = 0.075; % Altura veículo [m]
Lv = 0.19; % Largura veículo [m]
Dg = 0.06; % Distância entre centro de massa do giro e eixo de rotação [m]
Dv = 0.045; % Distância entre centro de massa do veículo e eixo de rotação
Omega = 6500*0.10472; % Velocidade de rotação do giro, rpm*conversão = rad/sec
g = 9.81; % Gravidade [m/s^2] 
IG11 = Mg*(Rg^2)/4 + Mg*(Ag^2)/12; % Algum momento de inercia
IG33 = Mg*(Rg^2)/2;
IB11 = Mv*(Av^2+Lv^2)/12; 

%% modelo

syms x1 x2 x3 u;
f1 = x3;
f2 = 0;
f3 = ((Mv*Dv+Mg*Dg)*g*sin(x1))/(IB11+Mv*(Dv^2)+IG11*(cos(x2)^2)+Mg*(Dg^2)+IG33*((sin(x2))^2));
f = [f1;f2;f3];
u1 = 0;
u2 = 1;
u3 = (-2*cos(x2)*sin(x2)*x3*(IG33-IG11)-Omega*cos(x2)*IG33)/(IB11+IG11*(cos(x2)^2)+Mv*(Dv^2)+Mg*(Dg^2)+IG33*(sin(x2)^2));
u=[u1;u2;u3];

A = double(subs(jacobian(f),[x1 x2],[0 0]));
B = double(subs(u, [x1 x2],[0 0]));
C = [1 0 0;
    0 1 0 ];
D = zeros(3,1);
x0 = [pi/180*180 0 0];
P1 = [-12.01 -12.02 -12];
K = place(A,B,P1);
Con = ctrb(A,B)
rank(Con)
Obs = obsv(A,C)
rank(Obs)
%% Controle realimentação de estados

% Verifica a controlabilidade do Sistema Aumentado
Con = ctrb(A,B)
vsCon = rank(Con)

r = .4;
R = r;
Q = eye(3);
Ka = lqr(A, B, Q, R)

polos = eig(A-B*Ka)

%% Observador


% Verifica a observabilidade da Planta
Obs = obsv(A,C)
vsObs = rank(Obs)

x0obs = [pi/4 ; 0; 0];  

pd = -10;
L=place(A',C',[pd pd-0.05 pd-0.03])';        % polo duplo de A-LC em s=-12     
L =lqr(A',C',eye(3),0.001*eye(2))'
%% Ruido
x0obs = [pi/180*45 ; 0; 0];  
x0 = [pi/180*45 0 0];

rl = 1;
V1 = 0.0001*eye(3);
V2 = 0.000005*eye(2);

Lkf = lqr(A',C',V1,rl*V2); 
Lkf = Lkf'
eig(A-Lkf*C)
L = Lkf;

%% PLOTS
%close all
% figure(1)
% plot(tout,x(:,2),tout,x(:,3),tout,x(:,4)) % plot estados
% grid
% title('Realimentação Estados')
% figure(2)
% plot(tout, y(:,2),tout,y(:,3))
% grid
% title('Realimentação de Estados')
% figure
% plot3(xe(2,:),xe(3,:),xe(4,:))
% figure
x0 = [pi/180*30 0 0];
hold on
plot3(xe1(2,:),xe1(3,:),xe1(4,:))
% figure(3)
% plot(tout,x1(:,2),'r-',tout,x1(:,3),'b-',tout,x1(:,4),'g-') % plot estados
% hold on
% plot(tout,x2(:,2),'r--',tout,x2(:,3),'b--',tout,x2(:,4),'g--') % plot estados
% hold off
% grid
% title('Controlador Observador')
% figure(4)
% plot(tout(1,, y1(:,2),tout,y1(:,3))
% grid
% title('Controlador-Observador');
figure(3)
plot(tout,xobs(:,5),'r-',tout,xobs(:,4),'b-') % plot estados
hold on
plot(tout,xobse(:,2),'r--',tout,xobse(:,3),'b--',tout,xobse(:,4),'g--') % plot estados
hold off
grid
legend('x1','x2','x3','x1e','x2e','x3e');
title('Controlador Observador')
figure(4)
plot(tout,yobs(:,2),tout,yobs(:,3))
grid
legend('y','u');
