%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Este arquivo foi construido
%%% a partir do pacote simbolico do MATLAB
%%%
%%% Notação :
%%%% theta = [th1 ; th];
%%%% d/dt (theta) = [dth1 ; dth2];
%%%% d/dt(y) = yp;
%%%% d/dt(yp) = ypp;
%%%% u = vetor de torques
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

y = [cos(th1)+cos(th1+th2);
     sin(th1)+sin(th1+th2)];

yp = [(-sin(th1)-sin(th1+th2))*dth1-sin(th1+th2)*dth2;
      (cos(th1)+cos(th1+th2))*dth1+cos(th1+th2)*dth2];

 
A = [(sin(th1)-sin(th1+2*th2))/(-3+cos(2*th2))  , (-sin(th1)+3*sin(th1+th2)-sin(th1-th2)+sin(th1+2*th2))/(-3+cos(2*th2));
     (-cos(th1)+cos(th1+2*th2))/(-3+cos(2*th2)) , (cos(th1)+cos(th1-th2)-3*cos(th1+th2)-cos(th1+2*th2))/(-3+cos(2*th2))];
 
a = [(-5*g+2*dth1^2*cos(th1)+4*cos(th1+th2)*dth1^2+8*dth1*cos(th1+th2)*dth2+4*cos(th1+th2)*dth2^2+2*dth1^2*cos(th1+2*th2)+3*g*cos(2*th2)+g*cos(2*th1+2*th2)+g*cos(2*th1))/(-6+2*cos(2*th2));
     (8*dth1*sin(th1+th2)*dth2+4*sin(th1+th2)*dth2^2+g*sin(2*th1)+2*dth1^2*sin(th1+2*th2)+g*sin(2*th1+2*th2)+g*sin(2*th2)+2*sin(th1)*dth1^2+4*sin(th1+th2)*dth1^2)/(-6+2*cos(2*th2))];

ypp = a + A*u;
