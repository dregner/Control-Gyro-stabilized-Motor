function [out] = geracao(x);
t = x;
A = 1;
B = 0;
C = 1;

yr1 = A*sin(t)+C;
yr2 = A*cos(t)+B;

dyr1 = A*cos(t);
dyr2 = -A*sin(t);

ddyr1 = -A*sin(t);
ddyr2 = -A*cos(t);

out = [yr1; dyr1; yr2; dyr2; ddyr1; ddyr2];