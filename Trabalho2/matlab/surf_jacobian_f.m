% Biosystem
% Workshop 7 

close all;
clc;

%% function f31 - jacobian(f,[x1 x2 x3])
f1= @(x) (5598529180295619*cos(x(1)))/(36028797018963968*((6276274086778753*cos(x(2))^2)/73786976294838206464 + (1560767486861519*sin(x(2))^2)/9223372036854775808 + 39669627053720337301/28823037615171174400000)); % minimization
n=110;
x1 = linspace(-8,8,n);
y1 = linspace(-8,8,n);
Z1=zeros(length(x1),length(y1));

for i=1:length(x1)
    for j=1:length(y1)
        Z1(i,j)=f1([x1(i) y1(j)]);
    end
end

%% function f32 - jacobian(f,[x1 x2 x3])
f2= @(x)-(34766114932442899370811494898981*cos(x(2))*sin(x(1))*sin(x(2)))/(1329227995784915872903807060280344576*((6276274086778753*cos(x(2))^2)/73786976294838206464 + (1560767486861519*sin(x(2))^2)/9223372036854775808 + 39669627053720337301/28823037615171174400000)^2);
n=110;
x2 = linspace(-4,4,n);
y2 = linspace(-4,4,n);
Z2=zeros(length(x2),length(y2));

for i=1:length(x2)
    for j=1:length(y2)
        Z2(i,j)=f2([x2(i) y2(j)]);
    end
end
%% Surface Graphic f31
figure (1)
surf(x1,y1,Z1)
colormap(hot(20))
title('jacobian(f) - f31');
xlabel('x1');
ylabel('x2');
zlabel('f(x1,x2)');

%% Surface Graphic f32
figure (2)
surf(x2,y2,Z2)
colormap(hot(20))
title('jacobian(f) - f32');
xlabel('x1');
ylabel('x2');
zlabel('f(x1,x2)');



