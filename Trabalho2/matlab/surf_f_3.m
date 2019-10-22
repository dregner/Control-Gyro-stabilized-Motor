%% function f32 - jacobian(f,[x1 x2 x3])

Mv = 0.152; % Massa do veículo sem giro [kg]
Mg = 0.15; % Massa do giro [kg]
Rg = 0.095/2; % Raio do giro [m]
Ag = 0.006; % Es  pessura giro [m]
Av = 0.075; % Altura veículo [m]
Lv = 0.19; % Largura veículo [m]
Dg = 0.06; % Distância entre centro de massa do giro e eixo de rotação [m]
Dv = 0.045; % Distância entre centro de massa do veículo e eixo de rotação
Omega = 7200*0.10472; % Velocidade de rotação do giro, rpm*conversão = rad/sec
g = 9.81; % Gravidade [m/s^2] 
IG11 = (Mg*(Rg^2)/4) + (Mg*(Ag^2)/12); % Algum momento de inercia
IG33 = Mg*(Rg^2)/2;
IB11 = Mv*(Av^2+Lv^2)/12; 
    
f3= @(x)((Mv*Dv+Mg*Dg)*g*sin(x(1)))/(IB11+Mv*(Dv^2)+IG11*(cos(x(2))^2)+Mg*(Dg^2)+IG33*((sin(x(2)))^2));
f = @(x) (-2*cos(x(2))*sin(x(2))*x(1)*(IG33-IG11)-Omega*cos(x(2))*IG33)/(IB11+IG11*(cos(x(2))^2)+Mv*(Dv^2)+Mg*(Dg^2)+IG33*(sin(x(2))^2))
n=110;
x2 = linspace(-4,4,n);
y2 = linspace(-4,4,n);
Z2=zeros(length(x2),length(y2));
u =zeros(length(x2),length(y2));

for i=1:length(x2)
    for j=1:length(y2)
        Z2(i,j)=f3([x2(i) y2(j)]);
        u(i,j) =f([x2(i) y2(j)]);
    end
end
%% Surface Graphic f31
figure
surf(x2,y2,Z2)
%colormap(hot(20))
title('f - f3');
xlabel('x1');
ylabel('x2');
zlabel('f(x1,x2)');
%% Surface Graphic f31
figure
surf(x2,y2,u)
%colormap(hot(20))
title('u - u3');
xlabel('x3');
ylabel('x2');
zlabel('u(x3,x2)');
