function [out] = yyponto(x)
th1 = x(1);
th2 = x(2);
dth1 = x(3);
dth2 = x(4);

y = [cos(th1)+cos(th1+th2);
     sin(th1)+sin(th1+th2)];

yp = [(-sin(th1)-sin(th1+th2))*dth1-sin(th1+th2)*dth2;
      (cos(th1)+cos(th1+th2))*dth1+cos(th1+th2)*dth2];
  
  out = [y(1);yp(1);y(2);yp(2)];