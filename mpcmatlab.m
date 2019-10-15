% Some figure formatting
set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex'); 
clear all

numx = [1];
numy = [0.2 1 0 0];
Gcont = tf(numx,numy,'InputDelay',0.05);
%[num den] = tfdata(G, 'v');
G_d = c2d(Gcont,0.1)
[num1 den1] = tfdata(G_d, 'v');
[A,B,C,D] = tf2ss(num1,den1)
x0 = [60;40;0];
Ts = 0.1;
% Optimal control solution for $N = 4$
I = eye(3);
a=A*B;
b=A*A*B;
c=A*A*A*B;
G1 = [0 0 0 0 ; 0 0 0 0 ; 0 0 0 0];
G2 = [B(1) 0 0 0 ; B(2) 0 0 0 ; B(3) 0 0 0];
G3 = [a(1) B(1) 0 0 ; a(2) B(2) 0 0 ; a(3) B(3) 0 0];
G4 = [b(1) a(1) B(1) 0 ; b(2) a(2) B(2) 0;b(3) a(3) B(3) 0];
G5 = [c(1) b(1) a(1) B(1);c(2) b(2) a(2) B(2);c(3) b(3) a(3) B(3)];
G = [G1;G2;G3;G4;G5];
H = [I;A;A^2;A^3;A^4];
Q = C'*C;
R = 0.001;
% Q = eye(2);
Pinf = dare(A,B,Q,R,zeros(3,1),eye(3) );
Kinf = inv(R+B'*Pinf*B)*B'*Pinf*A;
% A*X*A' - X + Q = 0; X = dlyap(A,Q)
P = dlyap( (A-B*Kinf)',Q+Kinf'*R*Kinf);
Qf = P;
Qbar = blkdiag(Q,Q,Q,Q,Qf);
Rbar = blkdiag(R,R,R,R);
M = G'*Qbar*G + Rbar;
% input bound: umin <= u <= umax
umin = -5.5;
umax = 3;
lb = [umin;umin;umin;umin];
ub = [umax;umax;umax;umax];
% Apply MPC steps
xVec(:,1) = x0;
yVec(1) = C*x0;
uVec = [];
for kk = 1:100
 alpha = G'*Qbar*H*xVec(:,kk);
 Usol = quadprog(M,alpha',[],[],[],[],lb,ub);
 uVec(kk) = Usol(1);
 xVec(:,kk+1) = A*xVec(:,kk) + B*uVec(kk);
 yVec(kk+1) = C*xVec(:,kk+1);
 Xsol(:,1) = xVec(:,kk);
 Xsol(:,2) = A*Xsol(:,1) + B*Usol(1);
 Xsol(:,3) = A*Xsol(:,2) + B*Usol(2);
 Xsol(:,4) = A*Xsol(:,3) + B*Usol(3);
 Ysol(1) = C*Xsol(:,1);
 Ysol(2) = C*Xsol(:,2);
 Ysol(3) = C*Xsol(:,3);
 Ysol(4) = C*Xsol(:,4);
end
uVec = [uVec uVec(end)];
tVec = [0:1:100];

figure;
subplot(3,2,1)
stairs(tVec,uVec,'LineWidth',1.5);
xlabel('time [seconds]')
grid on;
ylabel('u')
title('Desired Acceleration Input u')
ylim('auto')
hold on

subplot(3,2,4)
stairs(tVec,[0 0 1]*xVec,'LineWidth',1.5)
grid on;
xlabel('time [seconds]')
ylabel('$a$')
title('State $a$')
ylim('auto')
hold on

subplot(3,2,3)
stairs(tVec,[1 0 0]*xVec,'LineWidth',1.5)
grid on;
xlabel('time [seconds]')
ylabel('$d$')
title('State $d$')
ylim('auto')
hold on

subplot(3,2,2)
stairs(tVec,[0 1 0]*xVec,'LineWidth',1.5)
grid on;
xlabel('time [seconds]')
ylabel('$v$')
title('State $v$')
ylim('auto')
hold on

subplot(3,2,[5,6])
stairs(tVec,C*xVec,'LineWidth',1.5)
grid on;
xlabel('time [seconds]')
ylabel('y')
title('Vehicle Position y')
ylim('auto')
hold on

set(findall(gcf,'Type','line'),'LineWidth',1.5)
set(findall(gcf,'-property','FontSize'),'FontSize',11);