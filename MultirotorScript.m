clear all; clc; close all;

%Obtain the multirotor EOMs
dxdt = MultirotorEOM();

%Convert the sybmolic expression to function handle, for faster
%calculations. Symbolic functions in ode45 loop are notoriously slow.
ht = matlabFunction(dxdt);

%Obtain the steady-state input values of thrust and moments by solving the
%dxdt(9:12) as those are the terms containing the inputs
%Initial guess to start the solver
Roots = SteadyStateEOM(dxdt);

% Atmospheric and gravity parameters (Constant altitude: Sea level)
%rho = 1.225*0.00194032; % Density (slug/ft^3)
g = 9.81;        % Gravitational acceleration (m/s^2)

% Multirotor parameters (Replace with the right UAV)
m = .4;          % Mass (kg)
W = m*g;            % Weight (N)
Ixx = .033;         % Roll inertia (kg-m^2)
Iyy = .037;         % Pitch inertia (kg-m^2)
Izz = .064;         % Yaw inertia (kg-m^2)
Ixy = 0; Iyz = 0; Izx = 0; % Coupled inertia, (kg-m^2)


%Steady-state values at hover (From Section 3.3.1)
x_ss = 0; y_ss = 0; z_ss = -5; phi_ss = 0; theta_ss = 0; psi_ss = ...
    15*pi/180; u_ss = 0; v_ss = 0; w_ss = 0; p_ss = 0; q_ss = 0; r_ss = ...
    0; u_T_ss = -m*g; u_phi_ss = 0; u_psi_ss = 0; u_theta_ss = 0; 

%{
%Steady-state values at steady translation (From Section 3.3.1)
x_ss = 0; y_ss = 0; z_ss = -5; phi_ss = 0; theta_ss = 0; psi_ss = ...
    15*pi/180; u_ss = 0; v_ss = 0; w_ss = 0; p_ss = 0; q_ss = 0; r_ss = ...
    0; u_T_ss = -m*g; u_phi_ss = 0; u_psi_ss = 0; u_theta_ss = 0; 
%}
    
x_vec_ss = [x_ss; y_ss; z_ss; phi_ss; theta_ss; psi_ss; u_ss; v_ss; ...
    w_ss; p_ss; q_ss; r_ss];
u_vec_ss = Roots';

mass_inertia_vec = [m; g; Ixx; Iyy; Izz; Ixy; Iyz; Izx];

%Linearized state and input matrices around steady-state as well as the
%steady-state input vector 
[A, B, uSS] = LinearMultirotor(dxdt, [x_vec_ss; u_vec_ss; mass_inertia_vec]);

%Define the C and D matrices
C = eye(12); D = [];

%Put the matrices in state-space form in continuous time (CT)
sysCT = ss(A, B, C, D);

%%%%%%%%%%%Simulation parameters%%%%%%%%%%%%%%%%%%%%%
Ts = .01; %Sampling time, s
t_vec = 0:Ts:20; %Creating the time vector, s
ref_vec = ones(12, length(t_vec)); % 12-state references for tracking
%Outgoing and incoming delay (Bidirectional delay), secs
out_del = 1; in_del = 1; 
%DT time steps of delays
out_delDT = out_del/Ts; in_delDT = in_del/Ts;

%The steady state values given above are the initial values (that is, we
%are starting from steady motion)
x_vec0 = x_vec_ss;

%Reference states (what we want to track)
x_ref = 0; y_ref = 0; z_ref = -10; phi_ref = 0*pi/180; theta_ref = 0; ...
    psi_ref = 0*pi/180; u_ref = 0; v_ref = 0; w_ref = 0; p_ref = 0; ...
    q_ref = 0; r_ref = 0;

%Convert the reference states to vectors
ref_vec(1,:) = x_ref*ref_vec(1,:); ref_vec(2,:) = y_ref*ref_vec(2,:);
ref_vec(3,:) = z_ref*ref_vec(3,:); ref_vec(4,:) = phi_ref*ref_vec(4,:);
ref_vec(5,:) = theta_ref*ref_vec(5,:); ref_vec(6,:) = psi_ref*ref_vec(6,:);
ref_vec(7,:) = u_ref*ref_vec(7,:); ref_vec(8,:) = v_ref*ref_vec(8,:);
ref_vec(9,:) = w_ref*ref_vec(9,:); ref_vec(10,:) = p_ref*ref_vec(10,:);
ref_vec(11,:) = q_ref*ref_vec(11,:); ref_vec(12,:) = r_ref*ref_vec(12,:);

%Custom references
%Adding another step change in z at time t = 4 sec
ref_vec(3,4/Ts+1:end) = ref_vec(3,4/Ts+1:end)-10;
%Sinusoidal pitch reference command
%ref_vec(11,:) = 1*sin(t_vec);

%TEST Trajectory Generator!!!!!!!!!!!!%%%
[XD, UD] = DesiredTrajectoryGenerator(ht, t_vec, x_vec_ss, uSS, ...
    mass_inertia_vec)

%%%% LQR Controller %%%%
%LQR weighting matrices
Q = eye(12); R = eye(4);
%Find the CT LQR gain
Klqr = lqr(sysCT, Q, R);

%Apply outgoing delay to the references. Due to this delay between the
%groundstation and the UAV the references will reach the UAV after some
%delay
del_ref_vec = addDelay(t_vec, ref_vec, out_del, x_vec0);

%Compute the actual states using a nonlinear model
[thistNL, actual_states_NL] = ode45(@(tdum,xdum) ...
    NonlinearMultirotorTrajectoryGenerator(tdum, xdum, ht, Klqr, t_vec, ...
    del_ref_vec, [uSS; mass_inertia_vec]), t_vec, x_vec0); 
%[~, ulqrNL] = NonlinearMultirotorTrajectoryGenerator(thistNL, ...
%actual_states_NL, ht, Klqr, t_vec, ref_vec, [uSS; mass_inertia_vec]);

%{
%% TEST TEST TEST %%
actual_states_Test(:,1) = x_vec0; 
Ac = A-B*Klqr; Bc = B; Cc = C; Dc = D;
syscl = ss(Ac, Bc, Cc, Dc);

[y, t,x] = lsim(syscl, del_ref_vec', t_vec');
%%
%}

%Compute the actual states using a Linear model
[thistL, actual_states_L] = ode45(@(tdum,xdum) ...
    LinearMultirotorTrajectoryGenerator(tdum, xdum, A, B, Klqr, t_vec, ...
    del_ref_vec, [uSS; mass_inertia_vec]), t_vec, x_vec0); 

%Apply incoming delay to the outputs/states (y = C*x = x when C = I). This
%is the delay between the UAV and the groundstation
delayed_states_NL = addDelay(t_vec, actual_states_NL', in_del, x_vec0);
delayed_states_L = addDelay(t_vec, actual_states_L', in_del, x_vec0);

%Add Gaussian white noise in the measurement with zero mean and some
%variance. The covariances are assumed to be zero.
NF = .1; %factor to control noise values in all the states 
x_noise_var = NF*.5; y_noise_var = NF*.5; z_noise_var = NF*.5; 
phi_noise_var = NF*.01; theta_noise_var = NF*.01; psi_noise_var = NF*.01; 
u_noise_var = NF*.01; v_noise_var = NF*.01; w_noise_var = NF*.01; 
p_noise_var = NF*.01; q_noise_var = NF*.01; r_noise_var = NF*.01;

%Create the noise vectors with the given variances and zero-mean
numb = length(t_vec)+out_delDT +in_delDT;
x_noise = x_noise_var * randn(1, numb); 
y_noise = y_noise_var * randn(1, numb);
z_noise = z_noise_var * randn(1, numb); 
phi_noise = phi_noise_var * randn(1, numb); 
theta_noise = theta_noise_var * randn(1, numb); 
psi_noise = psi_noise_var * randn(1, numb); 
u_noise = u_noise_var * randn(1, numb); 
v_noise = v_noise_var * randn(1, numb); 
w_noise = w_noise_var * randn(1, numb); 
p_noise = p_noise_var * randn(1, numb);
q_noise = q_noise_var * randn(1, numb); 
r_noise = r_noise_var * randn(1, numb);

%This vector will be added to the linear and nonlinear trajectories
noise_vec = [x_noise; y_noise; z_noise; phi_noise; theta_noise; psi_noise;
    u_noise; v_noise; w_noise; p_noise; q_noise; r_noise];

%Add noise to the actual states
actual_states_L = actual_states_L + noise_vec(:,out_delDT+...
    1:end-in_delDT)';
actual_states_NL = actual_states_NL + noise_vec(:,out_delDT+...
    1:end-in_delDT)';

%Add noise to the delayed states
delayed_states_L = delayed_states_L + noise_vec(:,out_delDT+ ...
    in_delDT+1:end);
delayed_states_NL = delayed_states_NL + noise_vec(:,out_delDT+ ...
    in_delDT+1:end);

%Smith predictor outputs
[predicted_states_L, predicted_states_NL] = ...
    SmithPredictor(sysCT, t_vec, x_vec0, ref_vec, in_del, out_del, ...
    delayed_states_L, delayed_states_NL, Klqr);


%Kalman Predictor outputs
%%
%DT system for Kalman predictor
sysDT = c2d(sysCT, Ts, 'tustin');
ADT = sysDT.A; BDT = sysDT.B; CDT = sysDT.C; DDT = sysDT.D;
predicted_states_L_KP = KalmanPredictor(sysDT, t_vec, x_vec0, ref_vec, ...
    in_del, out_del, actual_states_L', actual_states_NL', ...
    delayed_states_L, delayed_states_NL, Klqr, uSS);

%%

%Plotting
LnWd = 2; %Specify linewidth for sub plots
y_ax_lim_pos = [-25 5]; %Specify y axis limits for position subplots
y_ax_lim_ang = [-5 20]; %Specify y axis limits for orientation subplots
y_ax_lim_torq = [-0.2 .05]; %Specify y axis limits for torque subplots

figure
subplot(6,1,1)
plot(t_vec, predicted_states_L(1,:), 'c', t_vec, delayed_states_L(1,:)...
    , 'b', t_vec, predicted_states_NL(1,:), 'm', t_vec,...
    delayed_states_NL(1,:), 'r', t_vec, ref_vec(1,:), 'k-', ...
    t_vec, del_ref_vec(1,:), 'k--', 'LineWidth', LnWd)
ylabel('North, x (m)')
ylim(y_ax_lim_pos)
legend('Smith Predictor Linear', 'Actual linear system', ...
    'Smith Predictor Nonlinear', 'Actual nonlinear system', ...
    'Actual reference', 'Delayed reference')
subplot(6,1,2)
plot(t_vec, predicted_states_L(2,:), 'c', t_vec, delayed_states_L(2,:)...
    , 'b', t_vec, predicted_states_NL(2,:), 'm', t_vec, ...
    delayed_states_NL(2,:), 'r', t_vec, ref_vec(2,:), 'k-', ...
    t_vec, del_ref_vec(2,:), 'k--', 'LineWidth', LnWd)
ylabel('East, y (m)')
ylim(y_ax_lim_pos)
% legend('Smith Predictor Linear', 'Actual linear system', ...
%     'Smith Predictor Nonlinear', 'Actual nonlinear system', ...
%     'Actual reference', 'Delayed reference')
subplot(6,1,3)
plot(t_vec, predicted_states_L(3,:), 'c', t_vec, delayed_states_L(3,:)...
    , 'b', t_vec, predicted_states_NL(3,:), 'm', t_vec, ...
    delayed_states_NL(3,:), 'r', t_vec, ref_vec(3,:), 'k-', ...
    t_vec, del_ref_vec(3,:), 'k--', 'LineWidth', LnWd)
ylabel('Down, z (m)')
ylim(y_ax_lim_pos)
% legend('Smith Predictor Linear', 'Actual linear system', ...
%     'Smith Predictor Nonlinear', 'Actual nonlinear system', ...
%     'Actual reference', 'Delayed reference')
subplot(6,1,4)
plot(t_vec, predicted_states_L(4,:)*180/pi, 'c', t_vec, ...
    delayed_states_L(4,:)*180/pi, 'b', t_vec, ...
    predicted_states_NL(4,:)*180/pi, 'm', t_vec, ...
    delayed_states_NL(4,:)*180/pi, 'r', t_vec, ref_vec(4,:)*180/pi, ...
    'k-', t_vec, del_ref_vec(1,:)*180/pi, 'k--', 'LineWidth', LnWd)
ylabel('Roll, \phi (deg)')
ylim(y_ax_lim_ang)
% legend('Smith Predictor Linear', 'Actual linear system', ...
%     'Smith Predictor Nonlinear', 'Actual nonlinear system', ...
%     'Actual reference', 'Delayed reference')
subplot(6,1,5)
plot(t_vec, predicted_states_L(5,:)*180/pi, 'c', t_vec, ...
    delayed_states_L(5,:)*180/pi, 'b', t_vec, ...
    predicted_states_NL(5,:)*180/pi, 'm', t_vec, ...
    delayed_states_NL(5,:)*180/pi, 'r', t_vec, ref_vec(5,:)*180/pi, ...
    'k-', t_vec, del_ref_vec(5,:)*180/pi, 'k--', 'LineWidth', LnWd)
ylabel('Pitch, \theta (deg)')
%ylim(y_ax_lim_ang)
ylim([-70 70])
% legend('Smith Predictor Linear', 'Actual linear system', ...
%     'Smith Predictor Nonlinear', 'Actual nonlinear system', ...
%     'Actual reference', 'Delayed reference')
subplot(6,1,6)
plot(t_vec, predicted_states_L(6,:)*180/pi, 'c', t_vec, ...
    delayed_states_L(6,:)*180/pi, 'b', t_vec, ...
    predicted_states_NL(6,:)*180/pi, 'm', t_vec, ...
    delayed_states_NL(6,:)*180/pi, 'r', t_vec, ref_vec(6,:)*180/pi, ...
    'k-', t_vec, del_ref_vec(6,:)*180/pi, 'k--', 'LineWidth', LnWd)
ylabel('Yaw, \psi (deg)')
ylim(y_ax_lim_ang)
% legend('Smith Predictor Linear', 'Actual linear system', ...
%     'Smith Predictor Nonlinear', 'Actual nonlinear system', ...
%     'Actual reference', 'Delayed reference')
xlabel('Time, t (s)')

%{
figure
subplot(4,1,1)
plot(t_vec, u_actual(1,:), 'LineWidth', LnWd)
ylabel('Thrust, u_T (N)')
subplot(4,1,2)
plot(t_vec, u_actual(2,:), 'LineWidth', LnWd)
ylabel('Roll torque, u_\phi (N-m)')
ylim(y_ax_lim_torq)
subplot(4,1,3)
plot(t_vec, u_actual(3,:), 'LineWidth', LnWd)
ylabel('Pitch torque, u_\theta (N-m)')
ylim(y_ax_lim_torq)
subplot(4,1,4)
plot(t_vec, u_actual(4,:), 'LineWidth', LnWd)
ylabel('Yaw torque, u_\psi (N-m)')
xlabel('Time, t(s)')
ylim(y_ax_lim_torq)
%}