syms th1 th2 dth1 dth2 g

M = [3+2*cos(th2)  1+cos(th2);
     1+cos(th2)    1];

H = [-dth2*(2*dth1+dth2)*sin(th2);
    dth1*dth1*sin(th2)]+[2*g*sin(th1)+g*sin(th1+th2);
    g*sin(th1+th2)];

M1 = inv(M);

C = M1*H

A = [dth1; dth2; C]
B = [zeros(2,2); M1]

h11 = [ - sin(th1 + th2) - sin(th1), -sin(th1 + th2), 0, 0];
%h12 = [-cos(th1+th2)+cos(th1)-dth1*cos(th1+th2), -cos(th1+th2)-dth1*cos(th1+th2),-sin(th1+th2),-1];
h12 = [-dth2*cos(th1+th2)-dth1*(cos(th1+th2)+cos(th1)),-dth2*cos(th1+th2)-dth1*cos(th1+th2),-sin(th1+th2)-sin(th1),-sin(th1+th2)];
Lfh1 = h11*A;
Lgh1 = h11*B;
Lf2h1 = h12*A;
LgLfh1 = h12*B;

h21 = [cos(th1+th2)+cos(th1),cos(th1+th2),0,0];
h22 = [-dth2*sin(th1+th2),-dth1*(sin(th1+th2)+sin(th1)),cos(th1+th2)+cos(th1),cos(th1+th2)]
Lfh2 = h21*A;
Lgh2 = h21*B;
Lf2h2 = h22*A
LgLfh2 = h22*B

Aa = [LgLfh1;LgLfh2]
aa = [Lfh1, Lf2h1;Lfh2,Lf2h2]