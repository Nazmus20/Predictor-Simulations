clear all; clc; close all;

M = 0.5;
m = 0.2;
b = 0.1;
I = 0.006;
g = 9.8;
l = 0.3;

p = I*(M+m)+M*m*l^2; %denominator for the A and B matrices

A = [0      1              0           0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0];
B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];
C = [1 0 0 0;
     0 0 1 0];
D = [0;
     0];
 
states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'x'; 'phi'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

Q = 1*C'*C; R = 1; K = lqr(A,B,Q,R)
%Q(1,1) = 0; Q(2,2) = 0; Q(3,3) = 0; Q(4,4) = 0; 

Ac = [(A-B*K)];
Bc = [-Ac];
Cc = [C];
Dc = [];%zeros(2,4);

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'r'};
outputs = {'x'; 'phi'};

sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);
%sys_cl = ss(Ac,B,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);

Ts = .01;
t = 0:Ts:50;
%r =[0.2;0]*ones(size(t));

r = [0;0;0;0]*ones(size(t));
%half_idx = floor(length(t)/2);
%r(2, half_idx:end) =  ones(size(half_idx:length(t)));
for i=1:length(t)-1
    r(1,i)= r(2,i)*t(i);
end
%r(1,:) = sin(t);

[y,t,x]=lsim(sys_cl,r,t);
%[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
%set(get(AX(1),'Ylabel'),'String','cart position (m)')
%set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
%title('Step Response with LQR Control')

%Steady-state conditions
x_star = [0; 0; 0; 0]; x_star_dot = [0; 0; 0; 0];
x_D = r; x_D_dot = [1;0;0;0]; uSS = 0;

u_D = uSS - inv(B'*B)*B'*((x_star_dot-x_D_dot)- A*(x_star-x_D));

X(:,1) = ones(4,1); 
for i = 1:length(t)
    u_tilde(i) = - K*(X(:,i)-x_D(:,i));
    X(:,i+1) = X(:,i) + Ts*(x_D_dot + A*(X(:,i)-x_D(:,i)) + B*u_tilde(i));
    Y(:,i) = C*X(:,i);
end

figure
plot(t, x(:,1), 'r', t, X(1,1:end-1), 'g--', t, r(1,:), 'k')
figure
plot(t, x(:,2), 'r', t, X(2,1:end-1), 'g--', t, r(2,:), 'k')
figure
plot(t, x(:,3), 'r', t, X(3,1:end-1), 'g--', t, r(3,:), 'k')
figure
plot(t, x(:,4), 'b', t, X(4,1:end-1), 'k--', t, r(4,:), 'k')
figure
plot(t, u_tilde)


%Create subsystem
As = A(2:4, 2:4); Bs = B(2:4);
Cs = [1, 0, 0; 0, 1, 0]; Ds = D;

Qs = Cs'*Cs; Rs = 1; Ks = lqr(As, Bs, Qs, Rs);

rs = [0;0;0]*ones(size(t))';
half_idxs = floor(length(t)/2);
rs(1, half_idxs:end) = ones(size(half_idxs:length(t)));
%r(1,:) = sin(t);

Xs(:,1) = zeros(3,1); s(1) = 0;
for i = 1:length(t)
    us_tilde(i) = - Ks*(Xs(:,i)-rs(:,i));
    Xs(:,i+1) = Xs(:,i) + Ts*(As*(Xs(:,i)-rs(:,i)) + Bs*us_tilde(i));
    Ys(:,i) = Cs*Xs(:,i);
    s(i+1) = s(i) + Ts*Xs(1,i);
end

figure
plot(t, Xs(1,1:end-1), 'b', t, rs(1,:), 'k')
figure
plot(t, Xs(2,1:end-1), 'b', t, rs(2,:), 'k')
figure
plot(t, Xs(3,1:end-1), 'b', t, rs(3,:), 'k')
figure
plot(t, us_tilde)
figure
plot(t, s(1:end-1))