%Script to run the UAV steady translation motion along xy-plane. That
%is, the UAV starts initially from hover, and starts translating along the
%xy-plane
clear all; clc; close all;

%Define the symbolic terms to be used
syms x y z phi_r theta_r psi_r u v w p q r u_T u_phi u_theta u_psi Ixx ...
    Iyy Izz Ixy Iyz Izx m g rho W Dx Dy Dz real

Ahover = [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
    0, 0, 0, 0, -9.81, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 9.81, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
Bhover = [0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0.666667, 0, 0, 0;
    0, 28.7356, 0, 0;
    0, 0, 21.7865, 0;
    0, 0, 0, 10.2354];


C = eye(12); D = [];

Q=eye(12); Q(4,4) = 1; Q(3,3) = 1; R = eye(4); Q(2,2)=1;


Klqr = lqr(Ahover,Bhover,Q,R); %LQR controller from hover motion

%Generate desired trajectory from linearized around translation motion
Atranslate = [0, 0, 0, 0, 0.117451, -2.12291, 1, 0, 0, 0, 0, 0;
    0, 0, 0, -0.117451, 0, 0, 0, 0.998113, -0.0613971, 0, 0, 0;
    0, 0, 0, 2.12291, 0, 0, 0, 0.0613971, 0.998113, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.998113, -0.0613971;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0613971, 0.998113;
    0, 0, 0, 0, -9.81, 0, -0.283461, 0, 0, 0, 0.0131111, 2.12612;
    0, 0, 0, 9.79149, 0, 0, 0, -0.566312, 0.00174795, -0.0131111, 0, 0;
    0, 0, 0, -0.602305, 0, 0, 0, 0.0034959, -0.566945, -2.12612, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

Btranslate = [0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0.666667, 0, 0, 0;
    0, 28.7356, 0, 0;
    0, 0, 21.7865, 0;
    0, 0, 0, 10.2354];



Ts = 1; %Sampling time, s
t_vec = 0:Ts:20; %Time vector, s

[des_state, des_input, Time] = DesiredTrajectoryGenerator2(Atranslate);

% IC = [0;0;-5;0;0;0;0;0;0;0;0;0];
% [ttest,xtest]= ode45(@(t,x) DesiredTrajectoryGenerator(t,x, Atranslate, Btranslate), t_vec, IC);
% figure
% plot(Time, delta_des_state(1,:))
% hold on
% plot(Time, delta_des_state(2,:))
% plot(Time, delta_des_state(3,:))
% 
% figure
% plot(Time, delta_des_state(4,:)*180/pi)
% hold on
% plot(Time, delta_des_state(5,:)*180/pi)
% plot(Time, delta_des_state(6,:)*180/pi)
% 
% figure
% plot(Time, delta_des_state(7,:))
% hold on
% plot(Time, delta_des_state(8,:))
% plot(Time, delta_des_state(9,:))

%%
%Outgoing and incoming delay (Bidirectional delay), secs
out_del = 1; in_del = 1; 
%DT time steps of delays
out_delDT = out_del/Ts; in_delDT = in_del/Ts;

%Resample the data followin the time vector
for iter = 1:length(t_vec)
    idx = find(Time -t_vec(iter) <= 1e-10,1,'last');
    XD(:,iter) = des_state(:,idx); 
    UD(:,iter) = des_input(:,idx);
%    XdotD(:,iter) = delta_des_state_deriv(:,idx);
end

des_state = XD ;
des_input = UD ;
%des_state_deriv = XdotD; %Because steady state is 0 vector

state0 = [0;0;-5;0;0;0;0;0;0;0;0;0];
input0 = [-14.715; 0; 0; 0];

%Apply outgoing delay to the references. Due to this delay
%between the groundstation and the UAV the references will reach the UAV 
%after some delay
del_des_state = addDelay(t_vec, des_state, out_del, state0);
del_des_input = addDelay(t_vec, des_input, out_del, input0);
%del_des_state_deriv=addDelay(t_vec, des_state_deriv, out_del, zeros(12,1));

delta_state0 = state0 - del_des_state(:,1);
delta_input0 = input0 - del_des_input(:,1);

%convert the symbolic variables to MATLAB finction handle 
%ht = matlabFunction(dxdtActual);
Klqr2 = lqr(Atranslate, Btranslate, Q, R);
%Use ode45() to integrate
%IC=[0;0;0;-1*pi/180;0;0;0;-1.13313;0.01978;0;0;0];

[thistL, delta_x] = ode45(@(tdum,xdum) ...
    LinearMultirotorTrajectory2(tdum, xdum, Atranslate, Btranslate, Klqr2, ...
    del_des_state-des_state, del_des_input-des_input, t_vec, in_del), ...
    t_vec, delta_state0); 
%vec = out_delDT+1:end;

% sol = dde23(@(tdum,xdum) LinearMultirotorTrajectoryDelay(tdum, xdum, Zdum, Ahover, Atranslate, Bhover, Btranslate, Klqr2, ...
%     del_des_state-steady_state, del_des_input-steady_input, des_state_deriv, t_vec), ...
%     out_del*ones(12,1),t_vec, IC); 
figure
plot(thistL, delta_x(:,2)+ del_des_state(2,:)', 'k-', t_vec, del_des_state(2,:), 'r--')

figure
plot(thistL, delta_x(:,3)+ del_des_state(3,:)', 'k-', t_vec, del_des_state(3,:), 'r--')

figure
plot(thistL, delta_x(:,4)+ del_des_state(4,:)', 'k-', t_vec, del_des_state(4,:), 'r--')

figure
plot(thistL, delta_x(:,8)+ del_des_state(8,:)', 'k-', t_vec, del_des_state(8,:), 'r--')

figure
plot(thistL, delta_x(:,9)+ del_des_state(9,:)', 'k-', t_vec, del_des_state(9,:), 'r--')

% AA = [Atranslate, zeros(12,12); -C, zeros(12,12)]; 
% BB = [Btranslate, zeros(12, 12); zeros(12,4), C]
% 
% 
% QQ = eye(24); RR = eye(16); QQ(3,3) = 10000;
% K = lqr(AA, BB, QQ, RR)