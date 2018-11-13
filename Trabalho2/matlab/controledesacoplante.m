function [out] = controledesacoplante(x)
th1 = x(1);
th2 = x(2);
dth1 = x(3);
dth2 = x(4);

v1 = x(5);
v2 = x(6);

d2y_1 = x(7);
d2y_2 = x(8);

g = 9.8;

d2y = [d2y_1; d2y_2];

v = [v1 v2]';

A = [(sin(th1)-sin(th1+2*th2))/(-3+cos(2*th2))  , (-sin(th1)+3*sin(th1+th2)-sin(th1-th2)+sin(th1+2*th2))/(-3+cos(2*th2));
     (-cos(th1)+cos(th1+2*th2))/(-3+cos(2*th2)) , (cos(th1)+cos(th1-th2)-3*cos(th1+th2)-cos(th1+2*th2))/(-3+cos(2*th2))];
 
a = [(-5*g+2*dth1^2*cos(th1)+4*cos(th1+th2)*dth1^2+8*dth1*cos(th1+th2)*dth2+4*cos(th1+th2)*dth2^2+2*dth1^2*cos(th1+2*th2)+3*g*cos(2*th2)+g*cos(2*th1+2*th2)+g*cos(2*th1))/(-6+2*cos(2*th2));
     (8*dth1*sin(th1+th2)*dth2+4*sin(th1+th2)*dth2^2+g*sin(2*th1)+2*dth1^2*sin(th1+2*th2)+g*sin(2*th1+2*th2)+g*sin(2*th2)+2*sin(th1)*dth1^2+4*sin(th1+th2)*dth1^2)/(-6+2*cos(2*th2))];

 
    if (det(inv(A)) == 0)
        u = v;
    else
        u = (inv(A))*((-a)+d2y+v);
    end
out = [u(1);u(2)];
