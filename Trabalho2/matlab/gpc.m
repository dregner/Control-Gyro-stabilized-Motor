clc
        clear all
        close all
%% NON LINEAR SYSTEM
        mV = 0.152;     % Massa do veículo, sem giroscópio [kg]
mG = 0.15;      % Massa do giroscópio [kg]
rG = 0.095/2;   % Raio do giroscópio [m]
aG = 0.006;     % Espessura do giroscópio [m]
aV = 0.075;     % Altura do veículo [m]
lV = 0.19;      % Comprimento do veículo [m]
dG = 0.06;      % Distância entre centro de massa do giroscópio e centro de rotação do veículo [m]
dV = 0.045;     % Distância entre centro de massa do veículo e o ponto de rotação do veículo [m]

omega = 6500*0.10472;   % Velocidade de rotação do giroscópio, RPM*conversão = rad/sec
g = 9.81;               % Gravidade [m/s^2]

IG11 = mG*(rG^2)/4 + mG*(aG^2)/12; % Momento de Inércia
        IG33 = mG*(rG^2)/2;
IB11 = mV*(aV^2+lV^2)/12;


%% LINEAR SYSTEM FOR TS = 5 ms
A=[1.0013 0 0.0050;
0 1.0000 0;
0.5319 0 1.0013];
B=[-0.001;
0.005;
-0.3943];
C=[1 0 0;
0 1 0;
0 0 1];
D=zeros(3,1);

Kd = dlqr(A, B, eye(3),1);

%% CONDICAO INICIAL

sim_step = 0.0001;
tsim = 5;
Ts = 0.001;
end_sim = ceil(tsim/sim_step);


x10 = pi/4;
x20 = 0;
x30 = 0;
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
num_f3 = (mV*dV+mG*dG)*g*sin(x1(k-1));
den_f3 = IB11+mV*dV^2+mG*dG^2+IG11*cos(x2(k-1))^2+IG33*sin(x2(k-1))^2;
f3 = num_f3/den_f3;

num_u3 = -2*cos(x2(k-1))*sin(x2(k-1))*(IG33-IG11)*x3(k-1)-omega*cos(x2(k-1))*IG33;
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
nP = 5;
nU = 5;
Q = 1*eye(3*nP);
R = 1*eye(nU);
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

for k = 1:end_sim
    
if k == 1
dt = dt+1;
x1(k) = x10;
x2(k) = x20;
x3(k) = x30;
x_v = [x1(k);x2(k);x3(k); 0];

else
dt = dt+1;
num_f3 = (mV*dV+mG*dG)*g*sin(x1(k-1));
den_f3 = IB11+mV*dV^2+mG*dG^2+IG11*cos(x2(k-1))^2+IG33*sin(x2(k-1))^2;
f3 = num_f3/den_f3;

num_u3 = -2*cos(x2(k-1))*sin(x2(k-1))*(IG33-IG11)*x3(k-1)-omega*cos(x2(k-1))*IG33;
den_u3 = den_f3;
u3 = num_u3/den_u3;

x1d(k) = x3(k-1);
x2d(k) = u(k-1);
x3d(k) = f3+u3*u(k-1);

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