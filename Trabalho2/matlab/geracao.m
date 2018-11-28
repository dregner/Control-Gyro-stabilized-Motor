function [out] = geracao(x);
t = x;
A = 0;
B = 1;
C = 1;
D = 0;
E = 0;
alfa = 2;
beta = 2;

if( t == 20)
    B = -1
    C = 1
end
if(t == 30)
    B = -1
    C = -1
end
if(t == 40)
    B = 1
    C= -1
end
% yr1 = A*sin(t)+C;
% yr2 = A*cos(t)+B;
% 
% dyr1 = A*cos(t);
% dyr2 = -A*sin(t);
% 
% ddyr1 = -A*sin(t);
% ddyr2 = -A*cos(t);

yr1 = A*sin(t)+D*(1-exp(-alfa*t))+C;
yr2 = A*cos(t)+E*(1-exp(-t*beta))+B;

dyr1 = A*cos(t)+D*alfa*exp(-alfa*t);
dyr2 = -A*sin(t)+E*beta*exp(-beta*t);

ddyr1 = -A*sin(t)-D*alfa^2*exp(-alfa*t);
ddyr2 = -A*cos(t)-E*beta^2*exp(-beta*t);

out = [yr1; dyr1; yr2; dyr2; ddyr1; ddyr2];