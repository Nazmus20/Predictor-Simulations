%Test script
clear all; clc; close all;

%Define the symbolic terms to be used
syms x y z phi_r theta_r psi_r u v w p q r u_T u_phi u_theta u_psi Ixx ...
    Iyy Izz Ixy Iyz Izx m g rho W Dx Dy Dz real

%Create a "Param" structure to hold the symbolic values (Makes it easy to
%pass to a function)
Param.m = m; Param.g = g; Param.rho = rho; Param.W = m*g; 
Param.aero = [Dx; Dy; Dz]; %Drag coefficients for aero forces
Param.I = [Ixx, Ixy Izx; Ixy, Iyy, Iyz; Izx, Iyz, Izz]; % Inertia matrix 
Param.states = [x; y; z; phi_r; theta_r; psi_r; u; v; w; p; q; r];
Param.inputs = [u_T; u_phi; u_theta; u_psi];

%Obtain the multirotor EOMs
dxdtActual = MultirotorEOMActual(Param);
dxdtModel =  MultirotorEOM(Param);
%{
%Convert the sybmolic expression to function handle, for faster
%calculations. Symbolic functions in ode45 loop are notoriously slow.
ht = matlabFunction(dxdt);
%}

%Obtain the steady-state input values of thrust and moments by solving the
%dxdt(9:12) as those are the terms containing the inputs
%Initial guess to start the solver
Roots = SteadyStateEOM(dxdtModel, Param);
    
%From Chen's paper on invariant EKF, adding drag coefficients
%Drag coefficients
Dx_val = .3265; Dy_val = .3265; Dz_val = 2*.3265;

% Multirotor parameters numerical values (Replace with the right UAV)
mass = 1.5;          % Mass [kg]
Inertia_xx = .0348;         % Roll inertia [kg-m^2]
Inertia_yy = .0459;         % Pitch inertia [kg-m^2]
Inertia_zz = .0977;         % Yaw inertia [kg-m^2]
Inertia_xy = 0; Inertia_yz = 0; Inertia_zx = 0; % Coupled inertia, [kg-m^2]
Inertia = [Inertia_xx, Inertia_xy Inertia_zx; Inertia_xy, Inertia_yy, ...
    Inertia_yz; Inertia_zx, Inertia_yz, Inertia_zz]; % Inertia matrix 
%All the UAV parameters are defined in a structure called 'Param'
% Atmospheric and gravity parameters (Constant altitude: Sea level)
density = 1.225; % Density [kg/m^3]
gravity = 9.81;        % Gravitational acceleration [m/s^2]
Weight = mass*gravity; % Weight [N]

%{
% Multirotor parameters numerical values (Replace with the right UAV)
mass = .4;          % Mass [kg]
Inertia_xx = .033;         % Roll inertia [kg-m^2]
Inertia_yy = .037;         % Pitch inertia [kg-m^2]
Inertia_zz = .064;         % Yaw inertia [kg-m^2]
Inertia_xy = 0; Inertia_yz = 0; Inertia_zx = 0; % Coupled inertia, [kg-m^2]
Inertia = [Inertia_xx, Inertia_xy Inertia_zx; Inertia_xy, Inertia_yy, ...
    Inertia_yz; Inertia_zx, Inertia_yz, Inertia_zz]; % Inertia matrix 
%All the UAV parameters are defined in a structure called 'Param'
% Atmospheric and gravity parameters (Constant altitude: Sea level)
rho = 1.225; % Density [kg/m^3]
gravity = 9.81;        % Gravitational acceleration [m/s^2]
Weight = mass*gravity; % Weight [N]
%}

%Create a "Values" structure to hold the numeric values (Makes it easy to
%pass to a function)
Values.m = mass; Values.g = gravity; Values.W = Weight; Values.rho = density;
Values.I = [Inertia_xx, Inertia_xy Inertia_zx; Inertia_xy, Inertia_yy, ...
    Inertia_yz; Inertia_zx, Inertia_yz, Inertia_zz]; % Inertia matrix 
Values.aero = [Dx_val; Dy_val; Dz_val]; %Values of the aero coefficients

%Steady-state values at hover (From Section 3.3.1)
x_ss = 0; y_ss = 0; z_ss = -5; phi_ss = 0; theta_ss = 0; psi_ss = ...
    0*pi/180; u_ss = 0; v_ss = 0; w_ss = 0; p_ss = 0; q_ss = 0; r_ss = ...
    0; u_T_ss = -m*g; u_phi_ss = 0; u_psi_ss = 0; u_theta_ss = 0;
    
%Put the steady-state values in the "Value" structure
Values.steady_states = [x_ss; y_ss; z_ss; phi_ss; theta_ss; psi_ss; ...
    u_ss; v_ss; w_ss; p_ss; q_ss; r_ss];

%Numerical values of 'Roots'
Roots_num = subs(Roots, [Param.m, Param.g, Param.I(1,1), Param.I(1,2), ...
    Param.I(1,3), Param.I(2,1), Param.I(2,2), Param.I(2,3), ...
    Param.I(3,1), Param.I(3,2), Param.I(3,3), Param.states'], [mass, ...
    gravity, Inertia(1,1), Inertia(1,2), Inertia(1,3), Inertia(2,1), ...
    Inertia(2,2), Inertia(2,3), Inertia(3,1), Inertia(3,2), ...
    Inertia(3,3), Values.steady_states']);

%Put the steady-state and input values in the "Value" structure
Values.steady_inputs = vpa(Roots_num)';

%Linearized state and input matrices around steady-state. This function can
%output both the values for A and B as symbolic or numeric matrices. Use
%symbolic values for theoretical formulations and numeric values for 
%simulations. Use the "model" EOM and not the "actual". So the controller
%will have some inaccuracy as it is derived from an inaccurate model.
[A, B] = LinearMultirotor(dxdtModel, Param, Values);

%Define the C and D matrices
C = eye(12); D = [];

%Put the matrices in state-space form in continuous time (CT). The matrices
%must be in the numeric form. Throw error if a symbolic matrix is used.
if isnumeric(A)==1 && isnumeric(B)==1
    sysCT = ss(A, B, C, D);
else
    disp('A and B matrices must be numeric')
    return
end

%Save the system to be used later in simulations
save("correctSystem.mat", "sysCT")

%%%%%%%%%%%Simulation parameters%%%%%%%%%%%%%%%%%%%%%
%Create a "Sim" structure to hold the following simulation values
Sim.Ts = .01; %Sampling time, [s]
Sim.t_vec = 0:Sim.Ts:20; %Creating the time vector, [s]
Sim.ref_vec = ones(12, length(Sim.t_vec)); % Reference vector for tracking

%Outgoing and incoming delay (Bidirectional delay), secs
Sim.out_del = 1; Sim.in_del = 1; 
%DT time steps of delays
Sim.out_delDT = Sim.out_del/Sim.Ts; Sim.in_delDT = Sim.in_del/Sim.Ts;

%The steady state values given above are the initial values (that is, we
%are starting from steady motion)
Sim.initial_states = Values.steady_states;

%{
%Custom references
%Adding another step change in z at time t = 4 sec
%ref_vec(3,4/Sim.Ts+1:end) = ref_vec(3,4/Sim.Ts+1:end)-10;
%Sinusoidal pitch reference command
%ref_vec(11,:) = 1*sin(t_vec);

%Create a NEW reference vector for the subsystem
%ref_vec_sub = ref_vec(3:12, :); % 10-state references for tracking excluding x and y
%}

%Known desired values are arranged like this: [x y z phi theta psi u v w p 
%q r u_T u_phi u_theta u_psi x_dot y_dot z_dot phi_dot theta_dot psi_dot 
%u_dot v_dot w_dot p_dot q_dot r_dot]' 
%These known desired values are the desired values that we know and want in
%the desired trajectory. For example for steady motion, all the
%acceleration terms should be 0. so u_dot = v_dot = w_dot = p_dot = q_dot =
%r_dot = 0. This lets us constrain some of the EOMs. The desired values
%that need to be solved for should be set as 'NaN'. Change the vector based
%on the flight condition (steady translation, hover etc.)

%For hover
%Desired/reference states/inputs/state derivatives (what we want to track)
x_D = nan; y_D = nan; z_D = -5; phi_D = 15*pi/180; theta_D = 0*pi/180; 
psi_D = 0*pi/180; u_D = 0; v_D = 0; w_D = 0; p_D = 0; q_D = 0; r_D = 0; 
u_TD = nan; u_phiD = nan; u_thetaD = nan; u_psiD = nan; x_dotD = 0; 
y_dotD = 0; z_dotD = 0; phi_dotD = 0*pi/180; theta_dotD = 0; 
psi_dotD = 0*pi/180; u_dotD = 0; v_dotD = 0; w_dotD = 0; p_dotD = 0; 
q_dotD = 0; r_dotD = 0; 


known_desired_vals = [x_D; y_D; z_D; phi_D; theta_D; psi_D; u_D; v_D; ...
    w_D; p_D; q_D; r_D; u_TD; u_phiD; u_thetaD; u_psiD; x_dotD; y_dotD; ...
    z_dotD; phi_dotD; theta_dotD; psi_dotD; u_dotD; v_dotD; w_dotD; ...
    p_dotD; q_dotD; r_dotD]; 

%TEST Trajectory Generator, Use the "actual" EOMs
[XD, X_dotD, UD] = DesiredTrajectoryGenerator(dxdtActual, Sim, Values, ...
    Param, known_desired_vals);



%Convert the reference states to vectors
Sim.des_state = XD; Sim.des_output = C*Sim.des_state;
Sim.des_input = UD; 
Sim.des_state_deriv = X_dotD; 

%Custom states
Sim.des_state = [0; 0; -10; 0; 0; 1*pi/180; 0; 0; 0; 0; 0; 0]*ones(size(Sim.t_vec));
Sim.des_input = [Values.m*Values.g; 0; 0; 0]*ones(size(Sim.t_vec));
Sim.des_state_deriv = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0]*ones(size(Sim.t_vec));
%Custom references
%Adding another step change in z at time t = 4 sec
Sim.des_state(3,4/Sim.Ts+1:end) = Sim.des_state(3,4/Sim.Ts+1:end)-10;

%%%% LQR Controller %%%%
%LQR weighting matrices
Q = 100000*eye(12); R = eye(4); Qsub = eye(10); Rsub = eye(4);
%Find the CT LQR gain
Klqr = lqr(sysCT, Q, R);

%Apply outgoing delay to the references. Due to this delay
%between the groundstation and the UAV the references will reach the UAV 
%after some delay
Sim.del_out_vec = addDelay(Sim.t_vec, Sim.des_state, Sim.out_del, ... 
    Sim.initial_states);

%convert the symbolic variables to MATLAB finction handle 
ht = matlabFunction(dxdtActual);
%[thistNL, actual_states_NL] = NonlinearMultirotorTrajectory(ht, Klqr, Values, Sim);


%Compute the actual states using a nonlinear model
[thistNL, actual_states_NL] = ode45(@(tdum,xdum) ...
    NonlinearMultirotorTrajectoryGenerator(tdum, xdum, ht, Klqr, ...
    Values, Sim), Sim.t_vec, Sim.initial_states); 
%Obtain the inputs from the same function (because ode45 outputs only the
%first value)
[~, ulqrNL] = NonlinearMultirotorTrajectoryGenerator(thistNL, ...
actual_states_NL, ht, Klqr, Values, Sim);

%%

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