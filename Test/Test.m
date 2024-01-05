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


steady_state = [0;0;-5;0;0;0;0;0;0;0;0;0];
steady_input = [-14.715; 0; 0; 0];

Ts = 1; %Sampling time, s
t_vec = 0:Ts:20; %Time vector, s

[delta_des_state, delta_des_input, delta_des_state_deriv, Time] = DesiredTrajectoryGenerator(Atranslate, Btranslate);

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
    XD(:,iter) = delta_des_state(:,idx); 
    UD(:,iter) = delta_des_input(:,idx);
    XdotD(:,iter) = delta_des_state_deriv(:,idx);
end

t_vec_add = 20+Ts:Ts:30;
for iter = 1:length(t_vec_add)
    XD_add(1:3, iter) = XD(1:3,end);
    XD_add(4:12, iter) = zeros(9,1);
    UD_add(:, iter) = steady_input;
    XdotD_add(:, iter) = zeros(12,1);
end


t_vec = [t_vec, t_vec_add];
XD = [XD, XD_add]; UD = [UD, UD_add]; XdotD = [XdotD, XdotD_add];

des_state = XD ;
des_input = UD ;
des_state_deriv = XdotD; %Because steady state is 0 vector

state0 = [0;0;-5;0;0;0;0;0;0;0;0;0];
input0 = [-14.715; 0; 0; 0];

%Apply outgoing delay to the references. Due to this delay
%between the groundstation and the UAV the references will reach the UAV 
%after some delay
del_des_state = addDelay(t_vec, des_state, out_del, state0);
del_des_input = addDelay(t_vec, des_input, out_del, input0);
del_des_state_deriv=addDelay(t_vec, des_state_deriv, out_del, zeros(12,1));

delta_state0 = state0 - del_des_state(:,1);
delta_input0 = input0 - del_des_input(:,1);

%convert the symbolic variables to MATLAB finction handle 
%ht = matlabFunction(dxdtActual);
Klqr2 = lqr(Atranslate, Btranslate, Q, R);
%Use ode45() to integrate
%IC=[0;0;0;-1*pi/180;0;0;0;-1.13313;0.01978;0;0;0];

[thistL, delta_x] = ode45(@(tdum,xdum) ...
    LinearMultirotorTrajectory(tdum, xdum, Ahover, Atranslate, Bhover, Btranslate, Klqr2, ...
    del_des_state-des_state, del_des_input-des_input, del_des_state_deriv, t_vec, in_del), ...
    t_vec, delta_state0); 
%vec = out_delDT+1:end;

% sol = dde23(@(tdum,xdum) LinearMultirotorTrajectoryDelay(tdum, xdum, Zdum, Ahover, Atranslate, Bhover, Btranslate, Klqr2, ...
%     del_des_state-steady_state, del_des_input-steady_input, des_state_deriv, t_vec), ...
%     out_del*ones(12,1),t_vec, IC); 
figure
plot(thistL, delta_x(:,2)+del_des_state(2,:)', 'k-', t_vec, del_des_state(2,:), 'r--')

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
%%



%%%%%%%%%%%Simulation parameters%%%%%%%%%%%%%%%%%%%%%
%Create a "Sim" structure to hold the following simulation values
Sim.Ts = 1; %Sampling time, [s]
Sim.t_vec = 0:Sim.Ts:20; %Creating the time vector, [s]

%Outgoing and incoming delay (Bidirectional delay), secs
Sim.out_del = 1; Sim.in_del = 1; 
%DT time steps of delays
Sim.out_delDT = Sim.out_del/Sim.Ts; Sim.in_delDT = Sim.in_del/Sim.Ts;

%The steady state values given above are the initial values (that is, we
%are starting from steady motion)
Sim.initial_state = Values.steady_states; 
Sim.initial_state_deriv = zeros(12,1); %Starts from hover: x_dot=0
Sim.initial_input = Values.steady_inputs; %Initial inputs are steady inputs

%Known desired values are arranged like this: [x y z phi theta psi u v w p 
%q r u_T u_phi u_theta u_psi x_dot y_dot z_dot phi_dot theta_dot psi_dot 
%u_dot v_dot w_dot p_dot q_dot r_dot]' 
%These known desired values are the desired values that we know and want in
%the desired trajectory. For example for steady motion, all the
%acceleration terms should be 0. so u_dot = v_dot = w_dot = p_dot = q_dot =
%r_dot = 0. This lets us constrain some of the EOMs. The desired values
%that need to be solved for should be set as 'NaN'. Change the vector based
%on the flight condition (steady translation, hover etc.)
known_desired_vals = [x_D; y_D; z_D; phi_D; theta_D; psi_D; u_D; v_D; ...
    w_D; p_D; q_D; r_D; u_TD; u_phiD; u_thetaD; u_psiD; x_dotD; y_dotD; ...
    z_dotD; phi_dotD; theta_dotD; psi_dotD; u_dotD; v_dotD; w_dotD; ...
    p_dotD; q_dotD; r_dotD]; 

%Desired trajectory generator, Use the "actual" EOMs
[XD, X_dotD, UD] = DesiredTrajectoryGenerator(dxdtActual, Sim, Values, ...
    Param, known_desired_vals);

fx = subs(fx, [Param.m, Param.g, Param.I(1,1), Param.I(1,2), ...
    Param.I(1,3), Param.I(2,1), Param.I(2,2), Param.I(2,3), ...
    Param.I(3,1), Param.I(3,2), Param.I(3,3), Param.rho, Param.aero(1), Param.aero(2), Param.aero(3)], [Values.m,...
    Values.g, Values.I(1,1), Values.I(1,2), Values.I(1,3), ...
    Values.I(2,1), Values.I(2,2), Values.I(2,3), Values.I(3,1), ...
    Values.I(3,2), Values.I(3,3), Values.rho, Values.aero(1), Values.aero(2), Values.aero(3)]);
gx = subs(gx, [Param.m, Param.g, Param.I(1,1), Param.I(1,2), ...
    Param.I(1,3), Param.I(2,1), Param.I(2,2), Param.I(2,3), ...
    Param.I(3,1), Param.I(3,2), Param.I(3,3)], [Values.m,...
    Values.g, Values.I(1,1), Values.I(1,2), Values.I(1,3), ...
    Values.I(2,1), Values.I(2,2), Values.I(2,3), Values.I(3,1), ...
    Values.I(3,2), Values.I(3,3)]);
%%%TESTING%%% 
[ex_dot, Acl, Bcl] = MultirotorError(Param, fx, gx, XD);

Acl = double(Acl); Bcl = double(Bcl);
%sysCT.A = Acl; sysCT.B = -Acl

% 
% %Test
% delta_xd_dot = X_dotD;
% delta_xd = (XD - Values.steady_states)
% delta_ud = inv(BD'*BD)*BD'*(delta_xd_dot - AD*delta_xd)
% %delta_ud = UD - Values.steady_inputs 
% U_vec = delta_ud + Values.steady_inputs


%Put the desired states, inputs and derivatives into "Sim" structure
Sim.des_state = XD; %Reference states to track
Sim.des_input = UD; 
Sim.des_state_deriv = X_dotD;

%%%% LQR Controller %%%%
%LQR weighting matrices
Q = eye(12); %Q(1,1) = 10000; 
%Q(2,2) = 100; 
Q(3,3) = 100000; 
%Q(4,4) = 100;
%Q(5,5) = 10000; Q(6,6) = 10000;
%Q(3,3) = 1000000;
%Q(9,9) = 100;
R = 1*eye(4);
%Find the CT LQR gain
Klqr = lqr(Acl, Bcl, Q, R);

%Apply outgoing delay to the references. Due to this delay
%between the groundstation and the UAV the references will reach the UAV 
%after some delay
Sim.del_des_state = addDelay(Sim.t_vec, Sim.des_state, Sim.out_del, ... 
    Sim.initial_state);
Sim.del_des_state_deriv = addDelay(Sim.t_vec, Sim.des_state_deriv, ...
    Sim.out_del, Sim.initial_state_deriv);
Sim.del_des_input = addDelay(Sim.t_vec, Sim.des_input, ...
    Sim.out_del, Sim.initial_input);

%convert the symbolic variables to MATLAB finction handle 
ht = matlabFunction(dxdtActual);

%Use ode45() to integrate
[thistL, delta_x] = ode45(@(tdum,xdum) ...
    LinearMultirotorTrajectory(tdum, xdum, Acl, Bcl, Klqr, Sim), ...
    Sim.t_vec, Sim.initial_state-Sim.initial_state, in_del); 
[thistNL, actual_states_NL] = ode45(@(tdum,xdum) ...
    NonlinearMultirotorTrajectory(tdum, xdum, ht, Klqr, ...
    Values, Sim), Sim.t_vec, Sim.initial_state); 

actual_states_L(1,:) = Sim.initial_state';uL(:,1) = Sim.initial_input;
uNL(:,1) = Sim.initial_input;
for i=2:length(thistL)
    actual_states_L(i,:) = delta_x(i,:) + Sim.del_des_state(:,i)';
    %[~, uL(:,i)] = LinearMultirotorTrajectory(thistL(i), delta_x(i,:)', A, B, Klqr, Sim);
    [~, uNL(:,i)] = NonlinearMultirotorTrajectory(thistNL(i), actual_states_NL(i,:)',  ht, Klqr, ...
    Values, Sim);
end

%Apply incoming delay to the outputs/states (y = C*x = x when C = I). This
%is the delay between the UAV and the groundstation
del_actual_states_NL = addDelay(Sim.t_vec, actual_states_NL', ...
    Sim.in_del+Sim.Ts, Sim.initial_state);
del_actual_states_L = addDelay(Sim.t_vec, actual_states_L', ...
    Sim.in_del+Sim.Ts, Sim.initial_state);

%Store the states as original measurement y = H*x = x when H = I
orig_measurement = del_actual_states_NL;
orig_time = Sim.t_vec;


%Figure with true values and 0 noise
n_plot=4;
y_axis_labels = {'x-axis position, x (m)', 'y-axis position, y (m)', ...
    'z-axis position, z (m)', 'Roll angle, \phi (rad)', ...
    'Pitch angle, \theta (rad)', 'Yaw angle, \psi (rad)', ...
    'x-axis velocity, u (m/s)', 'y-axis velocity, v (m/s)', ...
    'z-axis velocity, w (m/s)', 'Roll rate, p (rad/s)', ...
    'Pitch rate, q (rad/s)', 'Yaw rate, r (rad/s)'};

figure
plot(Sim.t_vec, Sim.des_state(n_plot,:), 'k-', Sim.t_vec, ...
    Sim.del_des_state(n_plot,:), 'k--', Sim.t_vec, ...
    actual_states_NL(:,n_plot), 'r-', Sim.t_vec, ...
    actual_states_L(:,n_plot), 'b--', Sim.t_vec, ...
    del_actual_states_NL(n_plot,:), 'm-', Sim.t_vec, ...
    del_actual_states_L(n_plot,:), 'c--')
legend('Input commanded', 'Input received', 'Actual system states NL', ...
    'Actual system states L', 'Delayed system states NL', 'Delayed system states L')
xlabel( 'Time, t (s)')
ylabel(y_axis_labels(n_plot))