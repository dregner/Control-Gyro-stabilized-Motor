% Biosystem
% Workshop 7 

% close all;
% clc;

%%
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

f1 = x3;
f2 = 0;
f3 = @(x) ((Mv*Dv+Mg*Dg)*g*sin(x(1)))/(IB11+Mv*(Dv^2)+IG11*(cos(x(2))^2)+Mg*(Dg^2)+IG33*((sin(x(2)))^2));
n=110;
x0 = linspace(-pi/2,pi/2,n);
y0 = linspace(-pi/2,pi/2,n);
Z0=zeros(length(x0),length(y0));

for i=1:length(x0)
    for j=1:length(y0)
        Z0(i,j)=f3([x0(i) y0(j)]);
    end
end

%% function f31 - jacobian(f,[x1 x2 x3])
f1= @(x) (5598529180295619*cos(x(1)))/(36028797018963968*((6276274086778753*cos(x(2))^2)/73786976294838206464 + (1560767486861519*sin(x(2))^2)/9223372036854775808 + 39669627053720337301/28823037615171174400000)); % minimization
n=110;
x1 = linspace(-pi/2,pi/2,n);
y1 = linspace(-pi/2,pi/2,n);
Z1=zeros(length(x1),length(y1));

for i=1:length(x1)
    for j=1:length(y1)
        Z1(i,j)=f1([x1(i) y1(j)]);
    end
end

%% function f32 - jacobian(f,[x1 x2 x3])
f2= @(x)-(34766114932442899370811494898981*cos(x(2))*sin(x(1))*sin(x(2)))/(1329227995784915872903807060280344576*((6276274086778753*cos(x(2))^2)/73786976294838206464 + (1560767486861519*sin(x(2))^2)/9223372036854775808 + 39669627053720337301/28823037615171174400000)^2);
n=110;
x2 = linspace(-pi,pi,n);
y2 = linspace(-pi,pi,n);
Z2=zeros(length(x2),length(y2));

for i=1:length(x2)
    for j=1:length(y2)
        Z2(i,j)=f2([x2(i) y2(j)]);
    end
end

%% Surface Graphic f31
figure (1)
surf(x0,y0,Z0)
%colormap(hot(20))
title('f');
xlabel('x1');
ylabel('x2');
zlabel('f(x1,x2)');

%% Surface Graphic f31
figure (2)
surf(x1,y1,Z1)
%colormap(hot(20))
title('jacobian(f) - f31');
xlabel('x1');
ylabel('x2');
zlabel('f(x1,x2)');

%% Surface Graphic f32
figure (3)
surf(x2,y2,Z2)
%colormap(hot(20))
title('jacobian(f) - f32');
xlabel('x1');
ylabel('x2');
zlabel('f(x1,x2)');



