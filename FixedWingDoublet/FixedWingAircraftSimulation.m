%   Fixed-Wing Aircraft Simulation in Wind
%   Invariant EKF implementation on MTD3 model
%   Zakia Ahmed
%   2023
clc; clear all; close all;

global e1 e2 e3 m rho g Inertia b c S ...
    Cj2 Cj Cj0 ...
    Cxalpha Cxde Cx0 Cxq Cxalpha2...
    Czalpha Czq Cz0 Czde...
    Cmalpha Cmq Cmde Cm0 ...
    Cybeta Cyp Cyr Cyda Cydr Cy0 ...
    Clbeta Clp Clda Clr Cldr Cl0...
    Cnbeta Cnr Cnda Cndr Cnp Cn0... 
    drpsEq PropDia nProp etaProp VEq deEq thetaEq 
    
% Basis vectors
e1=[1;0;0;];
e2=[0;1;0;];
e3=[0;0;1;];

% Gravity and density
g=9.81; % m/s^2
rho= 1.237; % kg/m^3

% Aircraft physical parameters
m= 3.311; % kg
c= 0.254; % m
b= 1.80; % m
S=0.457; % m^2
Ixx=0.319; % kg-m^2^2
Iyy=0.267; % kg-m^2^2
Izz=0.471; % kg-m^2^2
Ixz=0.024; % kg-m^2^2
Ixy=0;
Iyz=0;
Inertia=[Ixx -Ixy -Ixz; -Ixy Iyy -Iyz; -Ixz -Iyz Izz;];
PropDia= 0.254; % m
nProp=2; % number of props
etaProp= 0.90; % Prop efficiency

%% Parameter estimates

%Longitudinal parameter estimates 
Cj2= -0.13096; Cj= -0.04005; Cj0= 0.115918; 

Cxalpha2 = +3.173 + 0.119; Cxalpha= 0.282; Cxde= 0.051; Cx0= 0.009-0.4374;
Cxq = 0;

Czalpha= -4.436-0.015; Czq= -12.540; Cz0= -0.255; Czde = 0;

Cmq = -14.019; Cmalpha=-0.444-0.027; Cmq=-14.019; Cmde=-0.415; Cm0=0.008;    

%laterial-directional parameter estimates
Cy0 = 0; Cybeta=-0.410; Cyp=0.221; Cyr=0.230; Cyda=0.118; Cydr=0.136;

Cl0 = 0; Clbeta=-0.035-0.004; Clp= -0.386; Clda=-0.137; Clr = 0; Cldr = 0;

Cn0 = 0; Cnp =0 ; Cnbeta=0.083+0.02; Cnr=-0.119; Cnda=0.013; Cndr=-0.068;


%% Determining equilbrium

%Finding an Equilbrium condition for wings level flight, start with initial
%values
VEq=50; % Equilibrium forward speed, m/s
thetaEq=-1*pi/180; %Equilibrium pitch angle, rad
deEq=0; %Elevator deflection at equilibrium, rad 
drpsEq=220; %Propeller rotation per second at equilibrium, rps

EqSol=fsolve('Equilibrium',[VEq;thetaEq;deEq]);
VEq=EqSol(1)
thetaEq=EqSol(2)
deEq=EqSol(3)

%Initial Conditions
s0=[0;0;-100;]; %Position, m
Theta0=[0*pi/180;thetaEq*pi/180;0*pi/180]; % Attitude, rad
v0=[VEq*cos(thetaEq);0;VEq*sin(thetaEq)]; %Body axis velocity, m/s
omega0=[0;0;0];  % Body angular velocity, rad/s
%Vw0=[0;0;0]; %Von-karman wind, m/s
IC=[s0;Theta0;v0;omega0]; %Initial condition vector for ODE45

%To silmulate accurately, use high precision (small Ts) timestep for
%ode45()
tf=30; %Final time, sec
Ts=.1; %Sampling time, sec
tvec=[0:Ts:tf]'; %High precision time vector, sec

in_del = 1; %Delay between command and system
out_del = 1; %Delays between system and groundstation
in_delDT = 1/Ts; %Delay between command and system in DT
out_delDT = 1/Ts; %Delays between system and groundstation in DT
%% Linearization around the equilibrium, obtained from "EOMs.nb"
%Linearization of 9-states delta_X = [phi;theta;psi;u;v;w;p;q;r]; and
%delta_U = [da; de; dr]
A =[0, 0, 0, 0, 0, 0, 1, 0, .0210031;
    0, 0, 0, 0, 0, 0, 0, 1, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 1.00022;
    0, -9.80784, 0, -1.47158, 0, .620938, 0, -.381349, 0;
    9.80784, 0, 0, 0, -.635785, 0, .689782, 0, -17.8358;
    0, -.205995, 0, -.935573, 0, -6.92332, 0, 15.6872, 0;
    0, 0, 0, 0, -.98159, 0, -10.1034, 0, -.158714;
    0, 0, 0, .0480782, 0, -2.30003, 0, -8.69619, 0;
    0, 0, 0, 0, 1.97102, 0, -.514821, 0, -2.10958];

B = [0,	0,	0; 0, 0, 0; 0, 0, 0; 0, 1.43626, 0; 3.3231, 0, 3.83002;
    0, 0, 0; -72.0089, 0, -1.83008; 0, -36.8122, 0; .963259, 0, -24.3248];

C = eye(9); %Output matrix with 9-states
Cfull = eye(12); %Output matrix with 12 states
D = [];

sysCT = ss(A, B, C, D);
sysDT = c2d(sysCT, Ts);
%% Maneuver 
Maneuver='doublet';

if strcmp('none',Maneuver)
    %No maneuver
    tmaneuver=tvec; 
    onesec_steps=1/Ts;
    de_vec=deEq*ones(length(tmaneuver),1);

elseif strcmp('doublet',Maneuver)
    %Doublet start time (in sec), amplitude (in rad), and duration (in sec)
    tstart = 5; doubletAmp = 17*pi/180; duration = 2; tmaneuver=tvec; 
    for n=1:length(tvec)
        if tvec(n) >= tstart && tvec(n) < tstart+duration 
            de_vec(n) = deEq + doubletAmp; %rad
        elseif tvec(n) >= tstart+duration && tvec(n) < tstart+2*duration 
            de_vec(n) = deEq - doubletAmp; %rad
        else
            de_vec(n) = deEq;
        end
    end
    de_vec_del = addDelay(tvec, de_vec, in_del+Ts, deEq);
end

%% ODE solver 
[~,xNLund]=ode45(@(t,x) FixedWingEOM(t,x,tmaneuver,de_vec, Ts),tvec,IC);
[~,xLund]=ode45(@(t,x) FixedWingEOMLinear(t,x,sysCT.A,sysCT.B,tmaneuver,de_vec, Ts),tvec,IC);
[tNL,xNL]=ode45(@(t,x) FixedWingEOM(t,x,tmaneuver,de_vec_del, Ts),tvec,IC);
[tL,xL]=ode45(@(t,x) FixedWingEOMLinear(t,x,sysCT.A,sysCT.B,tmaneuver,de_vec_del, Ts),tvec,IC);

yNLund = Cfull*xNLund'; yLund = Cfull*xLund'; %Convert states to output
yNL = Cfull*xNL'; yL = Cfull*xL'; %Convert states to output

%Add delay in the output, Initial output: C*IC
yNL_del = addDelay(tvec, yNL, out_del, Cfull*IC);
yL_del = addDelay(tvec, yL, out_del, Cfull*IC);


figure
plot(tvec, de_vec*180/pi, 'k-', tvec, de_vec_del*180/pi, 'k--')
legend('\delta_e', 'Delayed \delta_{e}')
ylabel('Elevator deflection, \delta_e (deg)')
ylim([-25 20])
xlabel('Time, t (s)')

figure
plot(tvec, yNLund(1,:), 'k-', tvec, yNL(1,:), 'r-', tvec, yNL_del(1,:), 'b-')
legend('Undelayed', 'Input delay', 'Input+Output delay')
xlabel('Time, t (s)')
xlim([5, 15])
ylabel('Inertial north position, X (m)')

figure
plot(tvec, yNLund(3,:), 'k-', tvec, yNL(3,:), 'r-', tvec, yNL_del(3,:), 'b-')
legend('Undelayed', 'Input delay', 'Input+Output delay')
xlabel('Time, t (s)')
xlim([5, 15])
ylabel('Inertial north position, X (m)')

figure
plot(tvec, yNLund(1,:), 'k-', tvec, yNL(1,:), 'r-', tvec, yNL_del(1,:), 'b-')
legend('Undelayed', 'Input delay', 'Input+Output delay')
xlabel('Time, t (s)')
xlim([5, 15])
ylabel('Inertial north position, X (m)')

figure
plot(tvec, yNLund(3,:), 'k-', tvec, yNL(3,:), 'r-', tvec, yNL_del(3,:), 'b-')
legend('Undelayed', 'Input delay', 'Input+Output delay')
xlabel('Time, t (s)')
xlim([5, 15])
ylabel('Inertial north position, X (m)')

%{
figure
plot(tvec, yNLund(1,:), 'k-', tvec, yLund(1,:), 'k--', tvec, yNL(1,:), 'r-', tvec, yNL_del(1,:), 'r--', tvec, yL(1,:), 'b-', tvec, yL_del(1,:), 'b--')
legend('xLund', 'xNLund', 'xNL', 'xNL_del', 'xL', 'xL_del')

figure
plot(tvec, yNLund(3,:), 'k-', tvec, yLund(3,:), 'k--', tvec, yNL(3,:), 'r-', tvec, yNL_del(3,:), 'r--', tvec, yL(3,:), 'b-', tvec, yL_del(3,:), 'b--')
legend('zLund', 'zNLund','zNL', 'zNL_del', 'zL', 'zL_del')

figure
plot(tvec, yNLund(5,:), 'k-', tvec, yLund(5,:), 'k--', tvec, yNL(5,:), 'r-', tvec, yNL_del(5,:), 'b-', tvec, yL(5,:), 'r--', tvec, yL_del(5,:), 'b--')
legend('\thetaLund', '\thetaNLund','\thetaNL', '\thetaNL_del', '\thetaL', '\thetaL_del')
%}

%% Noise

%Add Gaussian white noise in the measurement with zero mean and some
%variance. The covariances are assumed to be zero.
NF = 1; %factor to control noise values in all the states 
x_noise_std = NF*.01; y_noise_std = NF*.01; z_noise_std = NF*.01; 
phi_noise_std = NF*.01; theta_noise_std = NF*.01; psi_noise_std = NF*.01; 
u_noise_std = NF*.1; v_noise_std = NF*.1; w_noise_std = NF*.1; 
p_noise_std = NF*.01; q_noise_std = NF*.01; r_noise_std = NF*.01;

%Create the noise vectors with the given variances and zero-mean
numb = length(tvec) + out_delDT + in_delDT;
x_noise = x_noise_std * randn(1, numb); 
y_noise = y_noise_std * randn(1, numb);
z_noise = z_noise_std * randn(1, numb); 
phi_noise = phi_noise_std * randn(1, numb); 
theta_noise = theta_noise_std * randn(1, numb); 
psi_noise = psi_noise_std * randn(1, numb); 
u_noise = u_noise_std * randn(1, numb); 
v_noise = v_noise_std * randn(1, numb); 
w_noise = w_noise_std * randn(1, numb); 
p_noise = p_noise_std * randn(1, numb);
q_noise = q_noise_std * randn(1, numb); 
r_noise = r_noise_std * randn(1, numb);

%This vector will be added to the linear and nonlinear trajectories
noise_vec = [x_noise; y_noise; z_noise; phi_noise; theta_noise; psi_noise;
    u_noise; v_noise; w_noise; p_noise; q_noise; r_noise];

%Add noise to the actual states
actual_output_L = yL + noise_vec(:, out_delDT + 1 : end - in_delDT);
actual_output_NL = yNL + noise_vec(:, out_delDT + 1 : end - in_delDT);

%Add noise to the delayed states
del_actual_output_L = yL_del + noise_vec(:, out_delDT+ in_delDT+1:end);
del_actual_output_NL = yNL_del + noise_vec(:, out_delDT+ in_delDT+1:end);

n_plot = 1;
%Figure with true values and some noise
figure
subplot(2,1,1)
plot(tvec, de_vec*180/pi, 'k-', tvec, de_vec_del*180/pi, 'r--')
xlabel('Time, t (s)')
ylabel('Elevator input, de (deg)')
subplot(2,1,2)
plot(tvec, actual_output_NL(n_plot,:), 'r-', tvec, ...
    actual_output_L(n_plot,:), 'b--', tvec, ...
    del_actual_output_NL(n_plot,:), 'm-', tvec, ...
    del_actual_output_L(n_plot,:), 'c--')
legend('actualNL', 'actualL', 'delayedNL', 'delayedL')
xlabel('Time, t (s)')
ylabel('Height, z (m)')

%Create the 3x1 input vector
Input_delayed = [zeros(size(de_vec_del)); de_vec_del; zeros(size(de_vec_del))];
Input_undelayed = [zeros(size(de_vec)); de_vec; zeros(size(de_vec))];
%Input_undelayed is the undelayed input vector which is sent to the
%predictor. The predictor internally acts upon the delayed and the
%undelayed inputs, so we don't need to send the delayed input separately.
%However, the delayed input was used in the ODE45 to get the UAV motion

%Start the predictors after 1 sec
idx = find(tvec==1.0);

t_vector = tvec;

%Initial condition at t = in_del. Because the states are at this time step
ICdel = yNLund(:, 1);

%%Predictor
[Time_pred_SP, YSP] = SmithPredictor(sysDT, tvec, del_actual_output_NL, Input_undelayed, VEq, thetaEq, deEq, IC, in_delDT, out_delDT, e1, e2, e3, Ts);
[Time_pred_KP, YKP, P] = KalmanPredictor(sysDT, t_vector, del_actual_output_NL(:,1:end), Input_undelayed(:,1:end), VEq, thetaEq, deEq, ICdel, in_delDT, out_delDT, e1, e2, e3, Ts);
[Time_pred_SP2, YSP2] = SmithPredictorNEW(sysDT, t_vector, del_actual_output_NL(:,1:end), Input_undelayed(:,1:end),Input_delayed(:,1:end), VEq, thetaEq, deEq, ICdel, in_delDT, out_delDT, e1, e2, e3, Ts);

%% Needed for EKP

ff = @(x,u) [x(9)*(sin(x(4))*sin(x(6)) + cos(x(4))*cos(x(6))*sin(x(5))) - 1.0*x(8)*(cos(x(4))*sin(x(6)) - 1.0*cos(x(6))*sin(x(4))*sin(x(5))) + x(7)*cos(x(5))*cos(x(6));
    x(8)*(cos(x(4))*cos(x(6)) + sin(x(4))*sin(x(5))*sin(x(6))) - 1.0*x(9)*(cos(x(6))*sin(x(4)) - 1.0*cos(x(4))*sin(x(5))*sin(x(6))) + x(7)*cos(x(5))*sin(x(6));
    x(9)*cos(x(4))*cos(x(5)) - 1.0*x(7)*sin(x(5)) + x(8)*cos(x(5))*sin(x(4));
    x(10) + x(12)*cos(x(4))*tan(x(5)) + x(11)*sin(x(4))*tan(x(5));
    x(11)*cos(x(4)) - 1.0*x(12)*sin(x(4));
    (x(12)*cos(x(4)) + x(11)*sin(x(4)))/cos(x(5));
    0.30202*(0.28272*x(7)^2 + 0.28272*x(8)^2 + 0.28272*x(9)^2)*(0.282*atan(x(9)/x(7)) + 0.051*u(2) + 3.292*atan(x(9)/x(7))^2 - 0.4284) - 9.81*sin(x(5)) - 0.097098*(x(7)^2 + x(8)^2 + x(9)^2)^(1/2) + x(8)*x(12) - 1.0*x(9)*x(11) - 0.0056818*x(7)^2 - 0.0056818*x(8)^2 - 0.0056818*x(9)^2 + 15.704;
    9.81*cos(x(5))*sin(x(4)) - 1.0*x(7)*x(12) + x(9)*x(10) + 0.30202*(0.28272*x(7)^2 + 0.28272*x(8)^2 + 0.28272*x(9)^2)*(0.118*u(1) + 0.136*u(3) - 0.41*asin(x(8)/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2)) + (0.1989*x(10))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2) + (0.207*x(12))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2));
    9.81*cos(x(4))*cos(x(5)) - 0.30202*(4.451*atan(x(9)/x(7)) + (1.5926*x(11))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2) + 0.255)*(0.28272*x(7)^2 + 0.28272*x(8)^2 + 0.28272*x(9)^2) + x(7)*x(11) - 1.0*x(8)*x(10);
    0.84021*x(11)*x(12) - 0.042813*x(10)*x(11) - 3.1469*(0.2466*u(1) + 0.0702*asin(x(8)/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2)) + (0.62532*x(10))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2))*(0.28272*x(7)^2 + 0.28272*x(8)^2 + 0.28272*x(9)^2) + 0.00016035*x(11)*(319.0*x(10) - 24.0*x(12)) + 3.1469*x(11)*(0.024*x(10) - 0.471*x(12)) + 0.16035*(0.28272*x(7)^2 + 0.28272*x(8)^2 + 0.28272*x(9)^2)*(0.0234*u(1) - 0.1224*u(3) + 0.1854*asin(x(8)/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2)) - (0.19278*x(12))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2));
    - 0.011236*x(10)*(8.0*x(10) - 157.0*x(12)) - 0.0037453*x(12)*(319.0*x(10) - 24.0*x(12)) - 3.7453*(0.28272*x(7)^2 + 0.28272*x(8)^2 + 0.28272*x(9)^2)*(0.11963*atan(x(9)/x(7)) + 0.10541*u(2) + (0.45222*x(11))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2) - 0.002032);
    0.042813*x(11)*x(12) - 0.56906*x(10)*x(11) + 0.00048105*x(11)*(8.0*x(10) - 157.0*x(12)) - 0.16035*(0.2466*u(1) + 0.0702*asin(x(8)/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2)) + (0.62532*x(10))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2))*(0.28272*x(7)^2 + 0.28272*x(8)^2 + 0.28272*x(9)^2) + 2.1313*x(11)*(0.319*x(10) - 0.024*x(12)) + 2.1313*(0.28272*x(7)^2 + 0.28272*x(8)^2 + 0.28272*x(9)^2)*(0.0234*u(1) - 0.1224*u(3) + 0.1854*asin(x(8)/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2)) - (0.19278*x(12))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2));];
 
 
FF = @(x,u) [0, 0, 0,       1.0*x(8)*(sin(x(4))*sin(x(6)) + 1.0*cos(x(4))*cos(x(6))*sin(x(5))) + x(9)*(cos(x(4))*sin(x(6)) - 1.0*cos(x(6))*sin(x(4))*sin(x(5))), cos(x(6))*(x(9)*cos(x(4))*cos(x(5)) - 1.0*x(7)*sin(x(5)) + x(8)*cos(x(5))*sin(x(4))), x(9)*(cos(x(6))*sin(x(4)) - 1.0*cos(x(4))*sin(x(5))*sin(x(6))) - 1.0*x(8)*(cos(x(4))*cos(x(6)) + 1.0*sin(x(4))*sin(x(5))*sin(x(6))) - 1.0*x(7)*cos(x(5))*sin(x(6)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          cos(x(5))*cos(x(6)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      cos(x(6))*sin(x(4))*sin(x(5)) - 1.0*cos(x(4))*sin(x(6)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          sin(x(4))*sin(x(6)) + cos(x(4))*cos(x(6))*sin(x(5)),                                                                                                             0,                                                                                                  0,                                                                                                                0;
            0, 0, 0, - 1.0*x(8)*(cos(x(6))*sin(x(4)) - 1.0*cos(x(4))*sin(x(5))*sin(x(6))) - 1.0*x(9)*(cos(x(4))*cos(x(6)) + 1.0*sin(x(4))*sin(x(5))*sin(x(6))), sin(x(6))*(x(9)*cos(x(4))*cos(x(5)) - 1.0*x(7)*sin(x(5)) + x(8)*cos(x(5))*sin(x(4))), 1.0*x(9)*(sin(x(4))*sin(x(6)) + 1.0*cos(x(4))*cos(x(6))*sin(x(5))) - 1.0*x(8)*(cos(x(4))*sin(x(6)) - 1.0*cos(x(6))*sin(x(4))*sin(x(5))) + x(7)*cos(x(5))*cos(x(6)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          cos(x(5))*sin(x(6)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          cos(x(4))*cos(x(6)) + sin(x(4))*sin(x(5))*sin(x(6)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      cos(x(4))*sin(x(5))*sin(x(6)) - 1.0*cos(x(6))*sin(x(4)),                                                                                                             0,                                                                                                  0,                                                                                                                0;
            0, 0, 0,                                                                                           cos(x(5))*(x(8)*cos(x(4)) - 1.0*x(9)*sin(x(4))),   - 1.0*x(7)*cos(x(5)) - 1.0*x(9)*cos(x(4))*sin(x(5)) - 1.0*x(8)*sin(x(4))*sin(x(5)),                                                                                                                                                                  0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               -1.0*sin(x(5)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          cos(x(5))*sin(x(4)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          cos(x(4))*cos(x(5)),                                                                                                             0,                                                                                                  0,                                                                                                                0;
            0, 0, 0,                                                                             (sin(x(5))*(x(11)*cos(x(4)) - 1.0*x(12)*sin(x(4))))/cos(x(5)),                                      (x(12)*cos(x(4)) + x(11)*sin(x(4)))/cos(x(5))^2,                                                                                                                                                                  0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            0,                                                                                                           1.0,                                                                                sin(x(4))*tan(x(5)),                                                                                              cos(x(4))*tan(x(5));
            0, 0, 0,                                                                                               - 1.0*x(12)*cos(x(4)) - 1.0*x(11)*sin(x(4)),                                                                                    0,                                                                                                                                                                  0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            0,                                                                                                             0,                                                                                          cos(x(4)),                                                                                                   -1.0*sin(x(4));
            0, 0, 0,                                                                                         (x(11)*cos(x(4)) - 1.0*x(12)*sin(x(4)))/cos(x(5)),                          (sin(x(5))*(x(12)*cos(x(4)) + x(11)*sin(x(4))))/cos(x(5))^2,                                                                                                                                                                  0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            0,                                                                                                             0,                                                                                sin(x(4))/cos(x(5)),                                                                                              cos(x(4))/cos(x(5));
            0, 0, 0,                                                                                                                                         0,                                                                      -9.81*cos(x(5)),                                                                                                                                                                  0,                                                                                                                                                                                                                                                                                                                                                                                                                        0.17077*x(7)*(0.282*atan(x(9)/x(7)) + 0.051*u(2) + 3.292*atan(x(9)/x(7))^2 - 0.4284) - (0.097098*x(7))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2) - 0.011364*x(7) - (1.0e+63*x(9)*(4.3921e+33*atan(x(9)/x(7)) + 1.8812e+32)*(x(7)^2 + x(8)^2 + x(9)^2))/(7.8125e+96*x(7)^2 + 7.8125e+96*x(9)^2),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              x(12) - 0.084523*x(8) - (0.097098*x(8))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2) + 0.56219*atan(x(9)/x(7))^2*x(8) + 0.0087095*u(2)*x(8) + 0.048158*atan(x(9)/x(7))*x(8),                                                                                                                                                                                                                                                                                                                                                                                                            0.17077*x(9)*(0.282*atan(x(9)/x(7)) + 0.051*u(2) + 3.292*atan(x(9)/x(7))^2 - 0.4284) - 1.0*x(11) - (0.097098*x(9))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2) - 0.011364*x(9) + (1.0e+63*x(7)*(4.3921e+33*atan(x(9)/x(7)) + 1.8812e+32)*(x(7)^2 + x(8)^2 + x(9)^2))/(7.8125e+96*x(7)^2 + 7.8125e+96*x(9)^2),                                                                                                             0,                                                                                          -1.0*x(9),                                                                                                             x(8);
            0, 0, 0,                                                                                                                  9.81*cos(x(4))*cos(x(5)),                                                            -9.81*sin(x(4))*sin(x(5)),                                                                                                                                                                  0,                                                                                                                                                                                                                                  0.17077*x(7)*(0.118*u(1) + 0.136*u(3) - 0.41*asin(x(8)/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2)) + (0.1989*x(10))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2) + (0.207*x(12))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2)) - 1.0*x(12) - (0.30202*(0.1989*x(7)*x(10)*(x(7)^2 + x(9)^2)^(1/2) - 0.41*x(7)*x(8)*(x(7)^2 + x(8)^2 + x(9)^2)^(1/2) + 0.207*x(7)*x(12)*(x(7)^2 + x(9)^2)^(1/2))*(0.28272*x(7)^2 + 0.28272*x(8)^2 + 0.28272*x(9)^2))/((x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2)^(3/2)),                                                                                                                                                                                                                                                                                                                                                          0.17077*x(8)*(0.118*u(1) + 0.136*u(3) - 0.41*asin(x(8)/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2)) + (0.1989*x(10))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2) + (0.207*x(12))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2)) - (0.30202*(0.41*(x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2)^(1/2) + 0.1989*x(8)*x(10) + 0.207*x(8)*x(12))*(0.28272*x(7)^2 + 0.28272*x(8)^2 + 0.28272*x(9)^2))/(x(7)^2 + x(8)^2 + x(9)^2)^(3/2),                                                                                                                                                                                                                                      x(10) + 0.17077*x(9)*(0.118*u(1) + 0.136*u(3) - 0.41*asin(x(8)/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2)) + (0.1989*x(10))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2) + (0.207*x(12))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2)) - (0.30202*(0.1989*x(9)*x(10)*(x(7)^2 + x(9)^2)^(1/2) - 0.41*x(8)*x(9)*(x(7)^2 + x(8)^2 + x(9)^2)^(1/2) + 0.207*x(9)*x(12)*(x(7)^2 + x(9)^2)^(1/2))*(0.28272*x(7)^2 + 0.28272*x(8)^2 + 0.28272*x(9)^2))/((x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2)^(3/2)),         x(9) + (0.060072*(0.28272*x(7)^2 + 0.28272*x(8)^2 + 0.28272*x(9)^2))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2),                                                                                                  0,        (0.062519*(0.28272*x(7)^2 + 0.28272*x(8)^2 + 0.28272*x(9)^2))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2) - 1.0*x(7);
            0, 0, 0,                                                                                                                 -9.81*cos(x(5))*sin(x(4)),                                                            -9.81*cos(x(4))*sin(x(5)),                                                                                                                                                                  0,                                                                                                                                                                                                                                                                                                                                                                                                                                     x(11) + 0.30202*((1.5926*x(7)*x(11))/(x(7)^2 + x(8)^2 + x(9)^2)^(3/2) + (4.451*x(9))/(x(7)^2*(x(9)^2/x(7)^2 + 1.0)))*(0.28272*x(7)^2 + 0.28272*x(8)^2 + 0.28272*x(9)^2) - 0.17077*x(7)*(4.451*atan(x(9)/x(7)) + (1.5926*x(11))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2) + 0.255),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           (0.13599*x(8)*x(11))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2) - 0.17077*x(8)*(4.451*atan(x(9)/x(7)) + (1.5926*x(11))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2) + 0.255) - 1.0*x(10),                                                                                                                                                                                                                                                                                                                                                                                                                                                    - 0.30202*(4.451/(x(7)*(x(9)^2/x(7)^2 + 1.0)) - (1.5926*x(9)*x(11))/(x(7)^2 + x(8)^2 + x(9)^2)^(3/2))*(0.28272*x(7)^2 + 0.28272*x(8)^2 + 0.28272*x(9)^2) - 0.17077*x(9)*(4.451*atan(x(9)/x(7)) + (1.5926*x(11))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2) + 0.255),                                                                                                     -1.0*x(8), x(7) - (0.481*(0.28272*x(7)^2 + 0.28272*x(8)^2 + 0.28272*x(9)^2))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2),                                                                                                                0;
            0, 0, 0,                                                                                                                                         0,                                                                                    0,                                                                                                                                                                  0, -(1.0*(0.1081*x(7)*asin(x(8)/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2))*(x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2)^(3/2) - 0.05405*x(7)*x(8)*(x(7)^2 + x(8)^2 + x(9)^2)^(3/2) - 0.55633*x(7)^3*x(10)*(x(7)^2 + x(9)^2)^(1/2) - 0.55633*x(7)*x(8)^2*x(10)*(x(7)^2 + x(9)^2)^(1/2) - 0.55633*x(7)*x(9)^2*x(10)*(x(7)^2 + x(9)^2)^(1/2) + 1.1127*x(7)*x(10)*(x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2) + 0.0087394*x(7)*x(12)*(x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2) + 0.43666*u(1)*x(7)*(x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2)^(3/2) + 0.011098*u(3)*x(7)*(x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2)^(3/2)))/((x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2)^(3/2)), 0.090667*x(8)*(0.0234*u(1) - 0.1224*u(3) + 0.1854*asin(x(8)/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2)) - (0.19278*x(12))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2)) - 1.7793*x(8)*(0.2466*u(1) + 0.0702*asin(x(8)/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2)) + (0.62532*x(10))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2)) - (3.1469*(0.0702*x(7)^2 + 0.0702*x(9)^2 - (0.62532*x(8)*x(10)*(x(7)^2 + x(9)^2)^(1/2))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2))*(0.28272*x(7)^2 + 0.28272*x(8)^2 + 0.28272*x(9)^2))/((x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2)) + (0.16035*(0.28272*x(7)^2 + 0.28272*x(8)^2 + 0.28272*x(9)^2)*(0.1854*x(7)^2 + 0.1854*x(9)^2 + (0.19278*x(8)*x(12)*(x(7)^2 + x(9)^2)^(1/2))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2)))/((x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2)), -(1.0*(0.1081*x(9)*asin(x(8)/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2))*(x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2)^(3/2) - 0.05405*x(8)*x(9)*(x(7)^2 + x(8)^2 + x(9)^2)^(3/2) - 0.55633*x(9)^3*x(10)*(x(7)^2 + x(9)^2)^(1/2) - 0.55633*x(7)^2*x(9)*x(10)*(x(7)^2 + x(9)^2)^(1/2) - 0.55633*x(8)^2*x(9)*x(10)*(x(7)^2 + x(9)^2)^(1/2) + 1.1127*x(9)*x(10)*(x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2) + 0.0087394*x(9)*x(12)*(x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2) + 0.43666*u(1)*x(9)*(x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2)^(3/2) + 0.011098*u(3)*x(9)*(x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2)^(3/2)))/((x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2)^(3/2)), 0.083863*x(11) - (1.9678*(0.28272*x(7)^2 + 0.28272*x(8)^2 + 0.28272*x(9)^2))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2),                                                                     0.083863*x(10) - 0.64581*x(12), - 0.64581*x(11) - (0.030912*(0.28272*x(7)^2 + 0.28272*x(8)^2 + 0.28272*x(9)^2))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2);
            0, 0, 0,                                                                                                                                         0,                                                                                    0,                                                                                                                                                                  0,                                                                                                                                                                                                                                                                                                                                                                                                                       3.7453*((0.45222*x(7)*x(11))/(x(7)^2 + x(8)^2 + x(9)^2)^(3/2) + (0.11963*x(9))/(x(7)^2*(x(9)^2/x(7)^2 + 1.0)))*(0.28272*x(7)^2 + 0.28272*x(8)^2 + 0.28272*x(9)^2) - 2.1177*x(7)*(0.11963*atan(x(9)/x(7)) + 0.10541*u(2) + (0.45222*x(11))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2) - 0.002032),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   (0.47884*x(8)*x(11))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2) - 2.1177*x(8)*(0.11963*atan(x(9)/x(7)) + 0.10541*u(2) + (0.45222*x(11))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2) - 0.002032),                                                                                                                                                                                                                                                                                                                                                                                                                              - 2.1177*x(9)*(0.11963*atan(x(9)/x(7)) + 0.10541*u(2) + (0.45222*x(11))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2) - 0.002032) - 3.7453*(0.11963/(x(7)*(x(9)^2/x(7)^2 + 1.0)) - (0.45222*x(9)*x(11))/(x(7)^2 + x(8)^2 + x(9)^2)^(3/2))*(0.28272*x(7)^2 + 0.28272*x(8)^2 + 0.28272*x(9)^2),                                                                                 0.56929*x(12) - 0.17978*x(10),      -(1.6937*(0.28272*x(7)^2 + 0.28272*x(8)^2 + 0.28272*x(9)^2))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2),                                                                                    0.56929*x(10) + 0.17978*x(12);
            0, 0, 0,                                                                                                                                         0,                                                                                    0,                                                                                                                                                                  0,   (0.028348*x(7)^3*x(10)*(x(7)^2 + x(9)^2)^(1/2) - 0.10853*x(7)*x(8)*(x(7)^2 + x(8)^2 + x(9)^2)^(3/2) + 0.21706*x(7)*asin(x(8)/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2))*(x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2)^(3/2) + 0.028348*x(7)*x(8)^2*x(10)*(x(7)^2 + x(9)^2)^(1/2) + 0.028348*x(7)*x(9)^2*x(10)*(x(7)^2 + x(9)^2)^(1/2) - 0.056696*x(7)*x(10)*(x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2) - 0.11616*x(7)*x(12)*(x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2) + 0.0058412*u(1)*x(7)*(x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2)^(3/2) - 0.14751*u(3)*x(7)*(x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2)^(3/2))/((x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2)^(3/2)), 1.2051*x(8)*(0.0234*u(1) - 0.1224*u(3) + 0.1854*asin(x(8)/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2)) - (0.19278*x(12))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2)) - 0.090667*x(8)*(0.2466*u(1) + 0.0702*asin(x(8)/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2)) + (0.62532*x(10))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2)) - (0.16035*(0.0702*x(7)^2 + 0.0702*x(9)^2 - (0.62532*x(8)*x(10)*(x(7)^2 + x(9)^2)^(1/2))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2))*(0.28272*x(7)^2 + 0.28272*x(8)^2 + 0.28272*x(9)^2))/((x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2)) + (2.1313*(0.28272*x(7)^2 + 0.28272*x(8)^2 + 0.28272*x(9)^2)*(0.1854*x(7)^2 + 0.1854*x(9)^2 + (0.19278*x(8)*x(12)*(x(7)^2 + x(9)^2)^(1/2))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2)))/((x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2)),   (0.028348*x(9)^3*x(10)*(x(7)^2 + x(9)^2)^(1/2) - 0.10853*x(8)*x(9)*(x(7)^2 + x(8)^2 + x(9)^2)^(3/2) + 0.21706*x(9)*asin(x(8)/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2))*(x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2)^(3/2) + 0.028348*x(7)^2*x(9)*x(10)*(x(7)^2 + x(9)^2)^(1/2) + 0.028348*x(8)^2*x(9)*x(10)*(x(7)^2 + x(9)^2)^(1/2) - 0.056696*x(9)*x(10)*(x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2) - 0.11616*x(9)*x(12)*(x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2) + 0.0058412*u(1)*x(9)*(x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2)^(3/2) - 0.14751*u(3)*x(9)*(x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2)^(3/2))/((x(7)^2 + x(9)^2)^(1/2)*(x(7)^2 + x(8)^2 + x(9)^2)^(3/2)), 0.11468*x(11) - (0.10027*(0.28272*x(7)^2 + 0.28272*x(8)^2 + 0.28272*x(9)^2))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2),                                                                     0.11468*x(10) - 0.083863*x(12), - 0.083863*x(11) - (0.41087*(0.28272*x(7)^2 + 0.28272*x(8)^2 + 0.28272*x(9)^2))/(x(7)^2 + x(8)^2 + x(9)^2)^(1/2)];
 
 hh = @(x,u) [x(1);
             x(2);
             x(3);
             x(4);
             x(5);
             x(6);
             x(7);
             x(8); 
             x(9);
             x(10);
             x(11);
             x(12)]; 
 
HH = @(x) eye(12);

W = 0.01*eye(12); 
V = 1*diag([x_noise_std^2; y_noise_std^2; z_noise_std^2; phi_noise_std^2;theta_noise_std^2;psi_noise_std^2;u_noise_std^2; v_noise_std^2;w_noise_std^2;p_noise_std^2;q_noise_std^2;r_noise_std^2]);

%%
global XdEq YdEq
vEq=[VEq; 0; 0;];
phieq=0;
psieq=0;
thetaeq=0;
RIBEq=expm(psieq*hat(e3))*expm(thetaeq*hat(e2))*expm(phieq*hat(e1));
XDotEq=RIBEq*vEq;
XdEq=XDotEq(1);
YdEq=XDotEq(2);
[TEKP, YEKP,] = EKP(tvec, del_actual_output_NL, Input_undelayed, ff, FF, hh, HH, W, V, IC, in_delDT, out_delDT, Ts);

%%
% close all;
% figure
% nplot=1;
% plot(tvec, del_actual_output_NL(nplot,:), 'r+', tvec, yNLund(nplot,:), 'k-', Time_pred_SP, YSP(nplot,:), 'r--', Time_pred_KP, YKP(nplot,:), 'b--', Time_pred_SP2, YSP2(nplot,:), 'g--', TEKP, YEKP(nplot,:), 'm--')
% legend('Act' , 'Undelayed measurement', 'YSP', 'YKP', 'YSP2','YEKP')

%close all;
figure
nplot=1;
label_vec = {'Inertial position, x (m)'; 'Inertial position, y (m)';
    'Inertial position, z (m)'; 'Euler Angles, \phi (rad)'; 'Euler Angles, \theta (rad)';
    'Euler Angles, \psi (rad)'; 'Body Velocity, u (m/s)'; 'Body Velocity, v (m/s)';
    'Body Velocity, w (m/s)'; 'Body Angular Velocity, p (rad/s)'; 
    'Body Angular Velocity, q (rad/s)'; 'Body Angular Velocity, r (rad/s)'};
plot(tvec, yNLund(nplot,:), 'k', Time_pred_SP, YSP(nplot,:), 'r--', Time_pred_KP, YKP(nplot,:), 'b--', TEKP, YEKP(nplot,:), 'g--', 'LineWidth', 2)
legend('Undelayed', 'SP', 'KP', 'EKP')
xlabel('Time, t (s)')
ylabel(label_vec{nplot})


%{
figure
plot(tvec, yNLund(1,:), 'k-', Time_pred_KP, YKP(1,:), 'b--')
legend('Undelayed measurement', 'YKP')




%% Plotting
close all

% figure(1)
% plot(t, drpsEq*ones(length(t)))
% legend('\delta_r_p_s')
% xlabel('time, seconds')
% ylabel('\delta_r_p_s')
% set(gca,'FontSize',14)

figure
set(gca,'FontSize',14)
subplot(2,1,1)
plot(tNL, xNL(:,1),'k-', tNL, xNL(:,2),'k--', tNL, xNL(:,3),'k-.', 'LineWidth',1)
hold on

xlabel('time, seconds')
ylabel('Position, m')
legend('x_a_c_t','y_a_c_t', 'z_a_c_t')

subplot(2,1,2)
plot(tNL, xNL(:,4)*180/pi,'k-', tNL, xNL(:,5)*180/pi,'k--', tNL, xNL(:,6)*180/pi,'k-.', 'LineWidth',1)
hold on

legend('\phi_a_c_t','\theta_a_c_t','\psi_a_c_t')
xlabel('time, seconds')
ylabel('Attitude, deg')

figure
set(gca,'FontSize',14)

subplot(3,1,1)
plot(tNL, xNL(:,7),'k-', 'LineWidth',1)
hold on

legend('u_a_c_t')
xlabel('time, seconds')
ylabel('Body velocity u, m/s')
minv=min(min([xNL(:,7) ]));
maxv=max(max([xNL(:,7) ]));
ylim([minv-.1 maxv+.1])
set(gca,'FontSize',14)

subplot(3,1,2)
plot(tNL, xNL(:,8),'k-', 'LineWidth',1)
hold on

minv=min(min([xNL(:,8)]));
maxv=max(max([xNL(:,8)]));
ylim([minv-.2 maxv+.2])
set(gca,'FontSize',14)
legend('v_a_c_t')
xlabel('time, seconds')
ylabel('Body velocity v, m/s')

subplot(3,1,3)
plot(tNL, xNL(:,9),'k-', 'LineWidth',1)
hold on

minv=min(min([xNL(:,9)]));
maxv=max(max([xNL(:,9)]));
ylim([minv-.2 maxv+.2])
set(gca,'FontSize',14)
legend('w_a_c_t')
xlabel('time, seconds')
ylabel('Body velocity w, m/s')

figure
subplot(3,1,1)
plot(tNL, xNL(:,10)*180/pi,'k-', 'LineWidth',1)
hold on
legend('p_a_c_t')
xlabel('time, seconds')
ylabel('Angular Velocity p, deg/s')
minomega=min(min([xNL(:,10)*180/pi]));
maxomega=max(max([xNL(:,10)*180/pi]));
ylim([minomega-2 maxomega+2])

subplot(3,1,2)
plot(tNL, xNL(:,11)*180/pi,'k-', 'LineWidth',1)
hold on

legend('q_a_c_t')
xlabel('time, seconds')
ylabel('Angular Velocity q, deg/s')
minomega=min(min([xNL(:,11)*180/pi]));
maxomega=max(max([xNL(:,11)*180/pi]));
ylim([minomega-2 maxomega+2])

subplot(3,1,3)
plot(tNL, xNL(:,12)*180/pi,'k-', 'LineWidth',1)
hold on

legend('r_a_c_t')
xlabel('time, seconds')
ylabel('Angular Velocity r, deg/s')
minomega=min(min([xNL(:,12)*180/pi]));
maxomega=max(max([xNL(:,12)*180/pi]));
ylim([minomega-2 maxomega+2])

% figure
% subplot(3,1,1)
% plot(t, xact(:,13),'k-', 'LineWidth',0.5)
% legend('Actual')
% xlabel('time, seconds')
% ylabel('Wind Velocity V_w_,_x, m/s')
% set(gca,'FontSize',14)
% 
% subplot(3,1,2)
% plot(t, xact(:,14),'k-', 'LineWidth',0.5)
% hold on
% 
% legend('Actual')
% xlabel('time, seconds')
% ylabel('Wind Velocity V_w_,_y, m/s')
% set(gca,'FontSize',14)
% 
% subplot(3,1,3)
% plot(t, xact(:,15),'k-', 'LineWidth',0.5)
% 
% legend('Actual')
% xlabel('time, seconds')
% ylabel('Wind Velocity V_w_,_z, m/s')
% set(gca,'FontSize',14)


%trajectory plot
addpath('TrajectoryPlots')
north=xNL(:,1);
east=xNL(:,2);
down=xNL(:,3);
roll=xNL(:,4);
pitch=xNL(:,5);
yaw=xNL(:,6);
scaleFactor=10;
var=8;
aircraft='cessna';
figure(6)
set(gca,'FontSize',12)
grid on
box on
ylabel('North, m'); 
xlabel('East, m');
zlabel('Down, m')
trajectory(north,east,down,roll,pitch,yaw,scaleFactor,var,aircraft)
ax = gca; ax.ZLim = [25 150];


close all
figure
plot(tNL, yNL_del(1:3,:),'-', tL, yL_del(1:3,:),'--')
legend('x NL','y NL', 'z NL', 'x L','y L', 'z L')
xlabel('time, seconds')
ylabel('Position, m')

figure
plot(tNL, yNL_del(4:6,:)*180/pi,'-', tL, yL_del(4:6,:)*180/pi,'--')
legend('\phi NL','\theta NL', '\psi NL', '\phi L','\theta L', '\psi L')
xlabel('time, seconds')
ylabel('Attitude, degrees')

figure
plot(tNL, yNL_del(7:9,:),'-', tL, yL_del(7:9,:),'--')
legend('u NL','v NL', 'w NL', 'u L','v L', 'w L')
xlabel('time, seconds')
ylabel('Velocity, m/s')

figure
plot(tNL, yNL_del(10:12, :)*180/pi,'-', tL, yL_del(10:12, :)*180/pi,'--')
legend('p NL','q NL', 'r NL', 'p L','q L', 'r L')
xlabel('time, seconds')
ylabel('Angular velocities, deg/s')
%}
