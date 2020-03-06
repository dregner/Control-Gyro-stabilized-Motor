% Biosystem
% Workshop 7 

% close all;
% clc;

% PARAMETROS DA SIMULAÇÃO GAZEBO

%% function f31 - jacobian(f,[x1 x2 x3])
f1= @(x)-(214861284644172625*cos(x(1)))/(4398046511104*(130*cos(x(2))^2 - 4753));
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
f2= @(x)-(13965983501871220625*cos(x(2))*sin(x(1))*sin(x(2)))/(1099511627776*(130*cos(x(2))^2 - 4753)^2);
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
figure (3)
surf(x1,y1,Z1)
%colormap(hot(20))
title('jacobian(f) - f31');
xlabel('x1');
ylabel('x2');
zlabel('f(x1,x2)');

%% Surface Graphic f32
figure (4)
surf(x2,y2,Z2)
%colormap(hot(20))
title('jacobian(f) - f32');
xlabel('x1');
ylabel('x2');
zlabel('f(x1,x2)');



