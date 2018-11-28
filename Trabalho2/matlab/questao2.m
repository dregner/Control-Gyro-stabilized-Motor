syms th1 th2 dth1 dth2 g

M = [3+2*cos(th2)  1+cos(th2);
     1+cos(th2)    1];

H = [-dth2*(2*dth1+dth2)*sin(th2);
    dth1*dth1*sin(th2)]+[2*g*sin(th1)+g*sin(th1+th2);
    g*sin(th1+th2)];

M1 = inv(M);

F = M1*H

F = [dth1; dth2; F]
G = [zeros(2,2); M1]
%%
h11 = [ - sin(th1 + th2) - sin(th1), -sin(th1 + th2), 0, 0];
%h12 = [-cos(th1+th2)+cos(th1)-dth1*cos(th1+th2), -cos(th1+th2)-dth1*cos(th1+th2),-sin(th1+th2),-1];
h12 = [-dth2*cos(th1+th2)-dth1*(cos(th1+th2)+cos(th1)),-dth2*cos(th1+th2)-dth1*cos(th1+th2),-sin(th1+th2)-sin(th1),-sin(th1+th2)];
Lfh1 = h11*F;
Lgh1 = h11*G;
Lf2h1 = h12*F;
LgLfh1 = h12*G;

h21 = [cos(th1+th2)+cos(th1),cos(th1+th2),0,0];
h22 = [-dth2*sin(th1+th2),-dth1*(sin(th1+th2)+sin(th1)),cos(th1+th2)+cos(th1),cos(th1+th2)]
Lfh2 = h21*F;
Lgh2 = h21*G;
Lf2h2 = h22*F
LgLfh2 = h22*G

Aa = [LgLfh1;LgLfh2]
aa = [Lfh1, Lf2h1;Lfh2,Lf2h2]

%%

y1 = cos(th1)+cos(th1+th2);
y2 = sin(th1)+sin(th1+th2);

% derivada de lee primeira saida

h11 = jacobian(y1,[th1,th2,dth1,dth2]);
Lfh1 = simplify(h11*F);
Lgh1 = h11*G;
h12 = jacobian(Lfh1,[th1,th2,dth1,dth2]);
Lf2h1 = simplify(h12*F); 
Lglfh1 = simplify(h12*G);

% derivada de lee segunda saida
h21 = jacobian(y2,[th1,th2,dth1,dth2]);
Lfh2 = simplify(h21*F);
Lgh2 = h21*G;
h22 = jacobian(Lfh2,[th1,th2,dth1,dth2]);
Lf2h2 = simplify(h22*F); 
Lglfh2 = simplify(h22*G);

al = simplify([Lfh1, Lf2h1; Lfh2, Lf2h2]);
Al = simplify([Lglfh1;Lglfh2]);

detA = simplify(det(Al));


%%

%dde = k1de + k0e

A = [0 1;
     0 0]
 B = [0;1];
 r = 0.01;
 K =lqr(A,B,eye(2),r)
 Kd = [-K,zeros(1,2);zeros(1,2) -K]
 