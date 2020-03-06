clc
        clear all
        close all
%% NON LINEAR SYSTEM
% Parâmetros do modelo
Mv = 0.152; % Massa do veículo sem giro [kg]
Mg = 0.15; % Massa do giro [kg]
Rg = 0.095/2; % Raio do giro [m]
Ag = 0.006; % Es  pessura giro [m]
Av = 0.075; % Altura veículo [m]
Lv = 0.19; % Largura veículo [m]
Dg = 0.06; % Distância entre centro de massa do giro e eixo de rotação [m]
Dv = 0.045; % Distância entre centro de massa do veículo e eixo de rotação
Omega = 6500*0.10472; % Velocidade de rotação do giro, rpm*conversão = rad/sec
g = 9.81; % Gravidade [m/s^2] 
IG11 = (Mg*(Rg^2)/4) + (Mg*(Ag^2)/12); % Algum momento de inercia
IG33 = Mg*(Rg^2)/2;
IB11 = Mv*(Av^2+Lv^2)/12; 
syms x1 x2 x3 u;
f1 = x3;
f2 = 0;
f3 = ((Mv*Dv+Mg*Dg)*g*sin(x1))/(IB11+Mv*(Dv^2)+IG11*(cos(x2)^2)+Mg*(Dg^2)+IG33*((sin(x2))^2));
f = [f1;f2;f3];
u1 = 0;
u2 = 1;
u3 = (-2*cos(x2)*sin(x2)*x3*(IG33-IG11)-Omega*cos(x2)*IG33)/(IB11+IG11*(cos(x2)^2)+Mv*(Dv^2)+Mg*(Dg^2)+IG33*(sin(x2)^2));
u=[u1;u2;u3];

%% modelo

Am = double(subs(jacobian(f),[x1 x2],[0 0]));
Bm = double(subs(u, [x1 x2],[0 0]));
Cm = [1 0 0; 0 1 0];% 0 0 0];
%Cm = [1 0 0;0 0 0];
Dmsim = zeros(2,1);
Ts = 0.005;
x0 = [pi/4 0 0];
    
[Amd, Bmd] = c2d(Am, Bm, Ts);
A = Amd;
B = Bmd;
C = eye(3);

Kd = dlqr(A, B, eye(3),1);

%% CONDICAO INICIAL

sim_step = 0.0001;
tsim = 5;
Ts = 0.001;
end_sim = ceil(tsim/sim_step);


x10 =x0(1);
x20 = x0(2);
x30 = x0(3);
u0 = 0;

dt = 0;
x1 = zeros(1,end_sim);
x2 = zeros(1,end_sim);
x3 = zeros(1,end_sim);
u = zeros(1,end_sim);
%% Simulacao LQR



for k=1:end_sim

if k == 1
dt = dt+1;
x1(k) = x10;
x2(k) = x20;
x3(k) = x30;

else
dt = dt+1;
num_f3 = (Mv*Dv+Mg*Dg)*g*sin(x1(k-1));
den_f3 = IB11+Mv*Dv^2+Mg*Dg^2+IG11*cos(x2(k-1))^2+IG33*sin(x2(k-1))^2;
f3 = num_f3/den_f3;

num_u3 = -2*cos(x2(k-1))*sin(x2(k-1))*(IG33-IG11)*x3(k-1)-Omega*cos(x2(k-1))*IG33;
den_u3 = den_f3;
u3 = num_u3/den_u3;

x1d(k) = x3(k-1);
x2d(k) = u(k-1);
x3d(k) = f3+u3*u(k-1);

x1(k) = x1(k-1) + (x1d(k-1)+x1d(k))*sim_step/2;
x2(k) = x2(k-1) + (x2d(k-1)+x2d(k))*sim_step/2;
x3(k) = x3(k-1) + (x3d(k-1)+x3d(k))*sim_step/2;
end


    if dt > Ts/sim_step
        dt = 0;
        x=[x1(k);x2(k);x3(k)];
        u(k) = -Kd*x;
    else
        if k == 1
            u(k) = u0;
        else
            u(k) = u(k-1);
        end
    end
        end



 
figure(1)
subplot(311)
plot(x1(1:end_sim));
subplot(312)
plot(x2(1:end_sim));
subplot(313)
plot(x3(1:end_sim));
% figure
% plot(u(1:end_sim));


%% Inicializa matriz T Q L -> ŷ = Q*x+L*u(t-1) + T*du
                                                 %
                                                 % Q = zeros(nx*nP, nx);
% L = zeros(nx*nP,nu);
% T = zeros(nx*nP,nu*nU);
% Q0 = C*A;
% L0 = C*B;
%
% for i = 1:nP
%     Q0 = Q0*A;
%     Q((i-1)*nx+(1:nx),:) = Q0;
% end
%
%
% for i = 1:nP
%     L0 = A*L0;
%     L((i-1)*nx+(1:nx),:) = L0;
% end
%
%
% for i = 1:nP
%     Tj = zeros(nx,nu*nU);
%     Ti = C;
%     for j = min(nU,i):-1:1
%         Tj(:,(j-1)*nu+(1:nu)) = Ti*B;
%         Ti = Ti*A;
%     end
%     T((i-1)*ny+(1:ny),:) = Tj;
% end
% Kp=(T'*Q*T+R)\(T'*R);
% size(Kp)
% K1=Kp(1:nu,:);



%% PARAMETROS CONTROLE
nP = 2;
nU = 1;
R = (10^(-4));
Q = [1 0 0 ; 0 1 0 ; 0 0 1];
%% Inicializa Matriz aumentada
[nx,nu]=size(B);
ny = size(C,1);
Am = [A B; zeros(nu,nx) eye(nu,nu)];
Bm = [B; zeros(nu,nu)];
Cm = [C zeros(ny,1)];
%% Inicializa matriz F e H ŷ = F(k-1)*x+H*du e MATRIZ K
F = zeros(ny*nP,nx+nu);
F0 = Cm;

for i = 1:nP
        F0 = F0*Am;
F((i-1)*ny+(1:ny),:) = F0;
end
        
H = zeros(ny*nP,nu*nU);
for i = 1:nP
        Hj = zeros(ny,nu*nU);
Hi = Cm;
for j = min(nU,i):-1:1
Hj(:,(j-1)*nu+(1:nu)) = Hi*Bm;
Hi = Hi*Am;
end
H((i-1)*ny+(1:ny),:) = Hj;
end
        
size(F)
size(H)

K=(H'*Q*H+R)\(H'*Q)
size(K)
K1=K(1:nu,:);
%% Inicializando DMC

x_v = zeros(3,size(Am,2));   % Inicializa vetor de saida
u_v = zeros(1,nu);          % Inicializa vetor de control

x_vet = zeros(4,end_sim);
u_vet = zeros(1,end_sim);
delta_u = zeros(1,end_sim);

%% MPC codigo

dt = 0;
x1 = zeros(1,end_sim);
x2 = zeros(1,end_sim);
x3 = zeros(1,end_sim);
u = zeros(1,end_sim);
[H1 F1 F2] = quad_pb_mat(A,B,eye(3),nP,Q,R);
K=(F1'*Q*F1+R)\(F1'*Q);
K1=K(1:nu,:);
for k = 1:end_sim
    
if k == 1
dt = dt+1;
x1(k) = x10;
x2(k) = x20;
x3(k) = x30;
x_v = [x1(k);x2(k);x3(k); 0];

else
dt = dt+1;
% num_f3 = (Mv*Dv+Mg*Dg)*g*sin(x1(k-1));
% den_f3 = IB11+Mv*Dv^2+Mg*Dg^2+IG11*cos(x2(k-1))^2+IG33*sin(x2(k-1))^2;
% f3 = num_f3/den_f3;
% 
% num_u3 = -2*cos(x2(k-1))*sin(x2(k-1))*(IG33-IG11)*x3(k-1)-Omega*cos(x2(k-1))*IG33;
% den_u3 = den_f3;
% u3 = num_u3/den_u3;
% 
% x1d(k) = x3(k-1);
% x2d(k) = u(k-1);
% x3d(k) = f3+u3*u(k-1);
x1d(k) = A(1,1)*x1(k-1)+A(1,2)*x2(k-1)+A(1,3)*x3(k-1)+B(1,1)*u(k-1);
x2d(k) = A(2,1)*x1(k-1)+A(2,2)*x2(k-1)+A(2,3)*x3(k-1)+B(2,1)*u(k-1);
x3d(k) = A(3,1)*x1(k-1)+A(3,2)*x2(k-1)+A(3,3)*x3(k-1)+B(3,1)*u(k-1);

x1(k) = x1(k-1) + (x1d(k-1)+x1d(k))*sim_step/2;
x2(k) = x2(k-1) + (x2d(k-1)+x2d(k))*sim_step/2;
x3(k) = x3(k-1) + (x3d(k-1)+x3d(k))*sim_step/2;


x_v = [x1(k); x2(k); x3(k); u(k-1)];

end

    if dt > Ts/sim_step
        dt = 0;
        delta_u(k) = K1*(zeros(ny*nP,1)-F*x_v);
        u(k) = delta_u(k) + u(k-1);
    else
        if k == 1
            u(k) = u0;
        else
            u(k) = u(k-1);
        end
    end
end

        figure(2)
subplot(311)
plot(x1(1:end_sim));
subplot(312)
plot(x2(1:end_sim));
subplot(313)
plot(x3(1:end_sim));
% figure
% plot(u(1:end_sim));