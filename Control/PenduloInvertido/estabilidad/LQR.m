load("linear_system.mat")
load("planta_motores.mat")
r = 7.5e-2;
Ts = 1/200;
sys_z = c2d(linsys1,Ts);

A = sys_z.A;
B = sys_z.B;
C = sys_z.C;

A_barra = [A        zeros(size(A,1),size(C,1));...
           -C*Ts    eye(size(C,1))];
B_barra = [B;...
            zeros(size(C,1),size(B,2))];

R = 0.01;
Q = [0 0 0;...
     0 1 0;...
     0 0 10000];

k = dlqr(A_barra,B_barra,Q,R);