function [out] = modelo(x);

g = 9.8;
th1 = x(1);
th2 = x(2);
dth1 = x(3);
dth2 = x(4);

tau1 = x(5);
tau2 = x(6);

theta = [th1 ; th2];

dtheta = [dth1 ; dth2];

tau = [tau1 ; tau2];

M = [3+2*cos(th2)  1+cos(th2);
     1+cos(th2)    1];

H = [-dth2*(2*dth1+dth2)*sin(th2);
    dth1*dth1*sin(th2)]+[2*g*sin(th1)+g*sin(th1+th2);
    g*sin(th1+th2)];

M1 = inv(M);

ddtheta = M1*[-H + tau];

out=[dtheta;ddtheta];
