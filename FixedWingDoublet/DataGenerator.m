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
s0=[0;0;-50;]; %Position, m
Theta0=[0*pi/180;thetaEq*pi/180;0*pi/180]; % Attitude, rad
v0=[VEq*cos(thetaEq);0;VEq*sin(thetaEq)]; %Body axis velocity, m/s
omega0=[0;0;0];  % Body angular velocity, rad/s
%Vw0=[0;0;0]; %Von-karman wind, m/s
IC=[s0;Theta0;v0;omega0]; %Initial condition vector for ODE45

%To silmulate accurately, use high precision (small Ts) timestep for
%ode45()
tf=40; %Final time, sec

Ts=1; %Sampling timefor simulation, sec

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
    tmaneuver=tvec_ode; 
    onesec_steps=1/Ts;
    de_vec=deEq*ones(length(tmaneuver),1);

elseif strcmp('doublet',Maneuver)
    %Doublet start time (in sec), amplitude (in rad), and duration (in sec)
    tstart = 5; doubletAmp = 20*pi/180; duration = 2; tmaneuver=tvec_ode; 
    for n=1:length(tvec_ode)
        if tvec_ode(n) >= tstart && tvec_ode(n) < tstart+duration 
            de_vec_ode(n) = deEq + doubletAmp; %rad
        elseif tvec_ode(n) >= tstart+duration && tvec_ode(n) < tstart+2*duration 
            de_vec_ode(n) = deEq - doubletAmp; %rad
        else
            de_vec_ode(n) = deEq;
        end
    end
    de_vec_del_ode = addDelay(tvec_ode, de_vec_ode, in_del+Ts_ode, deEq);
end

%% ODE solver 

[tNL,xNL]=ode45(@(t,x) FixedWingEOM(t,x,tmaneuver,de_vec_del_ode, Ts_ode),tvec_ode,IC);
[tL,xL]=ode45(@(t,x) FixedWingEOMLinear(t,x,sysCT.A,sysCT.B,tmaneuver,de_vec_del_ode, Ts_ode),tvec_ode,IC);

yNL_ode = Cfull*xNL'; yL_ode = Cfull*xL'; %Convert states to output

%Add delay in the output, Initial output: C*IC
yNL_ode_del = addDelay(tvec_ode, yNL_ode, out_del, Cfull*IC);
yL_ode_del = addDelay(tvec_ode, yL_ode, out_del, Cfull*IC);
save('Data.mat')

figure
plot(tvec_ode, de_vec_ode, 'k-', tvec_ode, de_vec_del_ode, 'k--', tvec_ode, yNL_ode(5,:), 'r-', tvec_ode, yNL_ode_del(5,:), 'b-', tvec_ode, yL_ode(5,:), 'r--', tvec_ode, yL_ode_del(5,:), 'b--')