clc
clear all
close all
%%
% Par�metros do modelo
Mv = 0.152; % Massa do ve�culo sem giro [kg]
Mg = 0.15; % Massa do giro [kg]
Rg = 0.095/2; % Raio do giro [m]
Ag = 0.006; % Espessura giro [m]
Av = 0.075; % Altura ve�culo [m]
Lv = 0.19; % Largura ve�culo [m]
Dg = 0.06; % Dist�ncia entre centro de massa do giro e eixo de rota��o [m]
Dv = 0.045; % Dist�ncia entre centro de massa do ve�culo e eixo de rota��o
Omega = 6500*0.10472; % Velocidade de rota��o do giro, rpm*convers�o = rad/sec
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
C = eye(3);
Dsim = zeros(3,1);
x0 = [pi/180*45 0 0];
P1 = [-12.01 -12.02 -12];
K = place(A,B,P1);
Con = ctrb(A,B)
rank(Con)
Obs = obsv(A,C)
rank(Obs)

%% DIstrecizacao Planta
Ts = 0.01;
[Ad, Bd] = c2d(A,B,Ts);

%% LQR for R.L.
Q = eye(3);
rf=0.7;
R=rf*eye(1);
Kd = dlqr(Ad,Bd,Q,R)
eig(Ad-Bd*Kd)
%Kd = place(Ad, Bd, [-0.7239 + 1.0219i,-0.7239 - 1.0219i,1])

%% Ganhos LQG para L
x0_obs = [pi/180*45 0 0];

Q = eye(3);
ro=0.1;
R=ro*eye(3)
Ld = dlqr(Ad',C',Q,R)'
eig(Ad-Ld*C)
V1 = 10e-7*eye(3);
V2 = 50e-7*eye(3);
rkf = 0.001;
Ldkf = dlqr(Ad',C', V1, rkf*V2)';
%Ldkf = place(Ad',C',[-10,-10.001,-10.01])';

%% PLOTS
% figure
% hold on
% plot(tout,x_din(:,2),'r')
% plot(tout,x_din(:,3),'b')
% plot(tout,x_din(:,4),'g')
% plot(tout,x_din(:,5),'r--')
% plot(tout,x_din(:,6),'b--')
% plot(tout,x_din(:,7),'g--')
%close all
% figure(1)
% plot(tout,x(:,2),tout,x(:,3),tout,x(:,4)) % plot estados
% grid
% title('Realimenta��o Estados')
% figure(2)
% plot(tout, y(:,2),tout,y(:,3))
% grid
% title('Realimenta��o de Estados')
% figure
% plot3(xe(2,:),xe(3,:),xe(4,:))
% figure
% x0 = [pi/180*30 0 0];
% hold on
% plot3(xe1(2,:),xe1(3,:),xe1(4,:))
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
% figure(3)
% plot(tout,xobs(:,5),'r-',tout,xobs(:,4),'b-') % plot estados
% hold on
% plot(tout,xobse(:,2),'r--',tout,xobse(:,3),'b--',tout,xobse(:,4),'g--') % plot estados
% hold off
% grid
% legend('x1','x2','x3','x1e','x2e','x3e');
% title('Controlador Observador')
% figure(4)
% plot(tout,yobs(:,2),tout,yobs(:,3))
% grid
% legend('y','u');
