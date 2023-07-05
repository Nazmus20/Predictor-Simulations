%Script to run the UAV hovering motion along z-axis and yaw-rotation. That
%is, the UAV starts initially from hover, goes to some desried altitude and
%yaw angle and then hovers again.
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

%Obtain the hovering inputs in symbolic form
Roots = HoverEOM(dxdtActual, Param);
   
%From Chen's paper on invariant EKF, adding drag coefficients
%Drag coefficients
Dx_val = .3265; Dy_val = .3265; Dz_val = 2*.3265;
Dx_val = 0; Dy_val = 0; Dz_val = 0; %For hover, they are set to 0.

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

%Create a "Values" structure to hold the actual numerical values of the
%parameters (These actual values are taken from Chen's invariant EKF paper)
Values.m = mass; Values.g = gravity; Values.W = Weight; Values.rho = density;
Values.I = [Inertia_xx, Inertia_xy Inertia_zx; Inertia_xy, Inertia_yy, ...
    Inertia_yz; Inertia_zx, Inertia_yz, Inertia_zz]; % Inertia matrix 
Values.aero = [Dx_val; Dy_val; Dz_val]; %Values of the aero coefficients

%Create a "wrong_values" structure to hold the estimated numerical values
%of the parameters (These wrong values are taken from J-M paper)
wrong_values.m = .4; %Mass [kg]
wrong_values.g = 9.81; %Gravitational acceleration [m/s^2] 
wrong_values.W = wrong_values.m*wrong_values.g; %Weight, [N] 
wrong_values.rho = 1.225; %Density [kg/m^3]
wrong_values.I = [.033, 0, 0; 0, .037, 0; 0, 0, .064]; %Inertia [kg-m^2]
wrong_values.aero = [0; 0; 0]; %Values of the aero coefficients

%Steady-state values at hover (From Section 3.3.1)
x_ss = 0; y_ss = 0; z_ss = -5; phi_ss = 0; theta_ss = 0; psi_ss = ...
    0*pi/180; u_ss = 0; v_ss = 0; w_ss = 0; p_ss = 0; q_ss = 0; r_ss = ...
    0; u_T_ss = -m*g; u_phi_ss = 0; u_psi_ss = 0; u_theta_ss = 0;
    
%Put the steady-state values in the "Value" structure
Values.steady_states = [x_ss; y_ss; z_ss; phi_ss; theta_ss; psi_ss; ...
    u_ss; v_ss; w_ss; p_ss; q_ss; r_ss];
%"wrong_values" should have the same steady states
wrong_values.steady_states = Values.steady_states; 
%Numerical values of 'Roots'
Roots_num = subs(Roots, [Param.m, Param.g, Param.I(1,1), Param.I(1,2), ...
    Param.I(1,3), Param.I(2,1), Param.I(2,2), Param.I(2,3), ...
    Param.I(3,1), Param.I(3,2), Param.I(3,3), Param.states'], [Values.m,...
    Values.g, Values.I(1,1), Values.I(1,2), Values.I(1,3), ...
    Values.I(2,1), Values.I(2,2), Values.I(2,3), Values.I(3,1), ...
    Values.I(3,2), Values.I(3,3), Values.steady_states']);

%Put the steady input values in the "Value" structure
Values.steady_inputs = double(Roots_num)';
%"wrong_values" should have the same steady inputs
wrong_values.steady_inputs = Values.steady_inputs;

%Linearized state and input matrices around steady-state. This function can
%output both the values for A and B as symbolic or numeric matrices. Use
%symbolic values for theoretical formulations and numeric values for 
%simulations.
[A, B] = LinearMultirotor(dxdtActual, Param, Values);

%Linearize the inaccurate model
[A_wrong, B_wrong] = LinearMultirotor(dxdtModel, Param, wrong_values);

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

%Put the matrices in state-space form in continuous time (CT). The matrices
%must be in the numeric form. Throw error if a symbolic matrix is used.
if isnumeric(A_wrong)==1 && isnumeric(B_wrong)==1
    sysCT_wrong = ss(A_wrong, B_wrong, C, D);
else
    disp('A_wrong and B_wrong matrices must be numeric')
    return
end

%Save the system to be used later in simulations
%save("correctSystem.mat", "sysCT")

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

%Reference states (what we want to track)
x_D = 0; y_D = 0; z_D = -10; phi_D = 0*pi/180; theta_D = 0; ...
psi_D = 0*pi/180; u_D = 0; v_D = 0; w_D = 0; p_D = 0; q_D = 0; ...
r_D = 0; 
%Reference inputs to solve for(They should be the same as the steady-state 
%inputs as we want to hover after achieving the desired states)
u_TD = nan; u_phiD = nan; 
u_thetaD = nan; u_psiD = nan; 
%Reference state derivatives (all are zeros for hover)
x_dotD = 0; y_dotD = 0; z_dotD = 0; phi_dotD = 0; theta_dotD = 0; 
psi_dotD = 0; u_dotD = 0; v_dotD = 0; w_dotD = 0; p_dotD = 0; q_dotD = 0; 
r_dotD = 0;

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

%Custom references
%Adding a step change in z at time t = 4 sec
XD(3,4/Sim.Ts+1:end) = XD(3,4/Sim.Ts+1:end)-10;
%Adding another step change in z at time t = 6 sec
%XD(3,6/Sim.Ts+1:end) = XD(3,6/Sim.Ts+1:end)-20;

%Put the desired states, inputs and derivatives into "Sim" structure
Sim.des_state = XD; %Reference states to track
Sim.des_input = UD; 
Sim.des_state_deriv = X_dotD;

%%%% LQR Controller %%%%
%LQR weighting matrices
Q = 1*eye(12); R = 1*eye(4);
%Find the CT LQR gain
Klqr = lqr(sysCT, Q, R);

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
[thistL, actual_states_L] = ode45(@(tdum,xdum) ...
    LinearMultirotorTrajectory(tdum, xdum, A, B, Klqr, Sim), ...
    Sim.t_vec, Sim.initial_state); 
[thistNL, actual_states_NL] = ode45(@(tdum,xdum) ...
    NonlinearMultirotorTrajectory(tdum, xdum, ht, Klqr, ...
    Values, Sim), Sim.t_vec, Sim.initial_state); 
for i=1:length(thistL)
    [~, uL(:,i)] = LinearMultirotorTrajectory(thistL(i), actual_states_L(i,:)', A, B, Klqr, Sim);
    [~, uNL(:,i)] = NonlinearMultirotorTrajectory(thistL(i), actual_states_NL(i,:)',  ht, Klqr, ...
    Values, Sim);
end

%Apply incoming delay to the outputs/states (y = C*x = x when C = I). This
%is the delay between the UAV and the groundstation
del_actual_states_NL = addDelay(Sim.t_vec, actual_states_NL', ...
    Sim.in_del, Sim.initial_state);
del_actual_states_L = addDelay(Sim.t_vec, actual_states_L', ...
    Sim.in_del, Sim.initial_state);

%Figure with true values and 0 noise
figure
plot(Sim.t_vec, Sim.des_state(3,:), 'k-', Sim.t_vec, ...
    Sim.del_des_state(3,:), 'k--', Sim.t_vec, ...
    actual_states_NL(:,3), 'r-', Sim.t_vec, ...
    actual_states_L(:,3), 'b--', Sim.t_vec, ...
    del_actual_states_NL(3,:), 'm-', Sim.t_vec, ...
    del_actual_states_L(3,:), 'c--')

%Add Gaussian white noise in the measurement with zero mean and some
%variance. The covariances are assumed to be zero.
NF = .1; %factor to control noise values in all the states 
x_noise_var = NF*.5; y_noise_var = NF*.5; z_noise_var = NF*.5; 
phi_noise_var = NF*.01; theta_noise_var = NF*.01; psi_noise_var = NF*.01; 
u_noise_var = NF*.1; v_noise_var = NF*.1; w_noise_var = NF*.1; 
p_noise_var = NF*.01; q_noise_var = NF*.01; r_noise_var = NF*.01;

%Create the noise vectors with the given variances and zero-mean
numb = length(Sim.t_vec)+Sim.out_delDT +Sim.in_delDT;
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
actual_states_L = actual_states_L' + noise_vec(:,Sim.out_delDT+...
    1:end-Sim.in_delDT);
actual_states_NL = actual_states_NL' + noise_vec(:,Sim.out_delDT+...
    1:end-Sim.in_delDT);

%Add noise to the delayed states
del_actual_states_L = del_actual_states_L + noise_vec(:,Sim.out_delDT+ ...
    Sim.in_delDT+1:end);
del_actual_states_NL = del_actual_states_NL + noise_vec(:,Sim.out_delDT+ ...
    Sim.in_delDT+1:end);

%Figure with true values and some noise
figure
%{
%Commeneted out for debugging
plot(Sim.t_vec, Sim.des_state(3,:), 'k-', Sim.t_vec, ...
    Sim.del_des_state(3,:), 'k--', Sim.t_vec, ...
    actual_states_NL(3,:), 'r-', Sim.t_vec, ...
    actual_states_L(3,:), 'b--', Sim.t_vec, ...
    del_actual_states_NL(3,:), 'm-', Sim.t_vec, ...
    del_actual_states_L(3,:), 'c--')
%}
%New plot for debugging
plot(Sim.t_vec, Sim.des_state(3,:), 'k-', Sim.t_vec, ...
    Sim.del_des_state(3,:), 'k--', Sim.t_vec, ...
    actual_states_NL(3,:), 'r-', Sim.t_vec, ...
    del_actual_states_NL(3,:), 'b-')
xlabel('Time, t (s)')
ylabel('Height, z, (m)')
legend('Current reference', 'Vehicle reference', 'Actual states', 'Measurements')

%The term "actual_states_NL"/"actual_states_L" are just to show that
%they are the same as the "del_actual_states_NL"/"del_actual_states_L". But
%the latter is the actual output of the system available to the
%groundstation. So, the "del_actual_states_NL"/"del_actual_states_L" will
%be used as the input to the predictors.

%Convert CT to DT
sysDT = c2d(sysCT, Sim.Ts, 'zoh');
sysDT_wrong = c2d(sysCT_wrong, Sim.Ts, 'zoh');


%Smith predictor outputs
[predicted_states_SP, CLTF_DT] = ...
    SmithPredictor(sysCT_wrong, Sim, del_actual_states_NL, Klqr);

%Figure with true trajectories and predicted trajectories. Predictor
%matches the actual when time is shifted by the delay:
%"Sim.t_vec-Sim.in_del-Sim.out_del-Sim.Ts"
figure
plot(Sim.t_vec, Sim.des_state(3,:), 'k-', Sim.t_vec, ...
    Sim.del_des_state(3,:), 'k--', Sim.t_vec, ...
    predicted_states_SP(3,:), 'r-', Sim.t_vec, ...
    del_actual_states_NL(3,:), 'b-')
legend('Current reference', 'Vehicle reference', 'Smith predictor', 'Measurements')
xlabel('Time, t (s)')
ylabel('Height, z, (m)')
%Append the Sim.t_vec, Sim.del_des_vec, Sim.del_des_input with 2 more
%seconds (required for Kalman predictor as it required previous n inputs
%for computation). During this pre-time frame no command is given. So, the
%desired inputs are the steady inputs and the desried states are the steady
%states

t_vec_append = -Sim.in_del-Sim.out_del:Sim.Ts:0-Sim.Ts;
for i=1:length(t_vec_append)
    des_input_append(:,i) = Values.steady_inputs;  
    del_des_input_append(:,i) = Values.steady_inputs;
    des_states_append(:,i) = Values.steady_states;
    del_des_states_append(:,i) = Values.steady_states;
    del_actual_states_NL_append(:,i) = Values.steady_states;
    uNL_append(:,i) = Values.steady_inputs;
end

%Create the noise vectors with the given variances and zero-mean
numb = length(t_vec_append);
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
%Now add noise to the actual states NL
del_actual_states_NL_append=del_actual_states_NL_append+noise_vec;

%Now append the vectors
del_actual_states_NL =[del_actual_states_NL_append, del_actual_states_NL];
Sim.t_vec = [t_vec_append, Sim.t_vec];
Sim.des_input = [des_input_append, Sim.des_input];
Sim.del_des_input = [del_des_input_append, Sim.del_des_input];
Sim.del_des_state = [del_des_states_append, Sim.del_des_state];
Sim.des_state = [des_states_append, Sim.des_state];
uNL = [uNL_append, uNL];

 %%  
 clear all; clc; close all;
 load workspace.mat %Temporary, load all workspace variable to reduce time
[predicted_states_KP] = ...
    KalmanPredictor(CLTF_DT, Sim, del_actual_states_NL);
[predicted_states_KP2] = ...
    KalmanPredictor2(CLTF_DT, Sim, del_actual_states_NL);
figure
%The two graphs match when the time is shifted by: "Sim.t_vec(1:end-1)-Sim.in_del+Sim.Ts" 
%plot(Sim.t_vec-Sim.in_del-Sim.out_del-Sim.Ts, del_actual_states_NL(3,:), 'k-', Sim.t_vec(1:end-1), predicted_states_KP(3,:), 'r--', Sim.t_vec(1:end-1), predicted_states_KP2(3,:), 'g--', Sim.t_vec(1+Sim.out_delDT+Sim.in_delDT:end), predicted_states_SP(3,:), 'b--')
plot(Sim.t_vec-Sim.in_del-Sim.out_del-Sim.Ts, del_actual_states_NL(3,:), 'k-', Sim.t_vec(1:end-1), predicted_states_KP(3,:), 'r--', Sim.t_vec(1+Sim.out_delDT+Sim.in_delDT:end), predicted_states_SP(3,:), 'b--')
title("Predictor comparison with truth data")
legend('Shifted measurement', 'KP', 'SP')
%legend('Actual', 'KP1', 'KP2', 'SP')
xlabel('Time, t (s)')
ylabel('Height, z (m)')

for i = 1:length(Sim.t_vec)-1
    uLQR(:,i) = Values.steady_inputs(1) -Klqr*(predicted_states_KP(:,i)-Sim.des_state(:,i));
end
figure
plot(Sim.t_vec(1:end-1), uLQR(1,:), 'k', Sim.t_vec-Sim.in_del-Sim.Ts, uNL(1,:), 'r')
%%
%{
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

%}