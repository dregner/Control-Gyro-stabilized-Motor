clear all
clc
%% MODELO SISTEMAA MOTO MULTIVAR

% Par�metros do modelo
Mv = 10+1+.2+0.2; % Massa do ve�culo sem giro [kg]
Mg = 3; % Massa do giro [kg]
Rg = 0.3; % Raio do giro [m]
Ag = 0.1; % Espessura giro [m]
Av = 0.4; % Altura ve�culo [m]
Lv = 1.2; % Largura ve�culo [m]
Dg = 0.5-0.05; % Dist�ncia entre centro de massa do giro e eixo de rota��o [m]
Dv = 0.1; % Dist�ncia entre centro de massa do ve�culo e eixo de rota��o
Omega = 150; % Velocidade de rota��o do giro, rpm*convers�o = rad/sec
g = 9.81; % Gravidade [m/s^2] 
IG11 = (Mg*(Rg^2)/4) + (Mg*(Ag^2)/12); % Algum momento de inercia
IG33 = Mg*(Rg^2)/2;
IB11 = Mv*(Av^2+Lv^2)/12; 
%syms Mv Mg Rg Ag Av Lv Dq Dv Omega g IG11 IG33 IB11
syms x1 x2 x3 u;
f1 = x3;
f2 = 0;
f3 = ((Mv*Dv+Mg*Dg)*g*sin(x1))/(IB11+Mv*(Dv^2)+IG11*(cos(x2)^2)+Mg*(Dg^2)+IG33*((sin(x2))^2));
f = [f1;f2;f3];
u1 = 0;
u2 = 1;
u3 = (-2*cos(x2)*sin(x2)*x3*(IG33-IG11)-Omega*cos(x2)*IG33)/(IB11+IG11*(cos(x2)^2)+Mv*(Dv^2)+Mg*(Dg^2)+IG33*(sin(x2)^2));
u=[u1;u2;u3];

%% modelo

Am = double(subs(jacobian(f),[x1 x2],[0 0]));
Bm = double(subs(u, [x1 x2],[0 0]));
Cm = [1 0 0; 0 1 0];% 0 0 0];
%Cm = [1 0 0;0 0 0];
Dmsim = zeros(2,1);
Tsm = 0.005;
Ts = Tsm;
x0m = [0 0 0];
    
[Amd, Bmd] = c2d(Am, Bm, Tsm);
A = Amd;
B = Bmd;
C = Cm;

Kd = dlqr(Amd, Bmd,eye(3),0.7);
eig(Amd - Kd*Bmd)
Lkf = dlqr(Amd',Cm',1e-4*eye(3),1*eye(2))'
eig(Amd-Lkf*Cm)
%% Filtro de Kalman
% Ruidos
w = 10e-8*eye(3); % estados
v = 30e-8*eye(2); % saida
w(1,1) = 10e-4*w(1,1);
w(2,2) = 10e-4*w(2,2);
    %Pam = 10e-6*eye(3); % Matriz covariancia
    %Pam = [2.939e-6 0 3.031e-5; 0 1.732e-8 0; 3.031e-5 0 0.0003126]
    Pam = [5.591e-6 0 5.765e-5; 0 1.732e-8 0; 5.765e-5 0 0.0005945];
%% Variaveis Manipulaveis
% Condi��o atual do Filtro / Moto

x0_filtro=[pi/4 0 0];
x0m = [0 0 0];
%Ruido w e v

w = 1e-4*eye(3);% w(1,1) = 1e-0*w(1,1); w(2,2) = 1e-0*w(2,2); w(3,3) = 1e-0*w(2,2);
v = 1e-7*eye(2);% v(1,1) = 1e-0*v(1,1); v(2,2) = 1e-0*v(2,2);
Pam = 1e8*eye(3); % Matriz covariancia

Q = 1e-70;
R = 1e-6;

% Ruidos para extended Kalman 
%Filter

w1 = R*eye(3); w1(1,1) = 6*w1(1,1); w1(2,2) = 3*w1(2,2); w1(3,3) = 10*w1(2,2);
v1= Q*eye(2); v1(1,1) = 2*v1(1,1); v1(2,2) = 3*v1(2,2);
% Saturacoes observador EKF
sat_x1o = 10000;
sat_x2o = 10000;
sat_x3o = 10000;