function [Time_pred, Y_pred] = ...
    SmithPredictor(sysDT, Time, Measurement, Input, VEq, thetaEq, deEq, ICdel, ICund, in_delDT, out_delDT, e1, e2, e3, Ts)

% SmithPredictor.m
%
% Smith Predictor simulation

%%%INPUTS%%%
% sysCT: The continuous time (CT) system of the UAV
% Klqr: The LQR controller gain
% Sim.t_vec: Time_vector of the simulation [s]
% x_vec0: Initial states for the simulation to start. The UAV is assumed to
% start from its steady-states
% ref_vec: The actual reference vector without any delay
% in_delDT: The value of the delay in the outgoing signal from the
% groundstation to the UAV, seconds
% out_delDT: The value of the delay in the incoming signal from the UAV to
% the groundstation, seconds
% uSS: Steady state input values. As the simulation starts from
% steady-state these are also the initial values of the input thrusts and
% torques
% isLinear: Boolean. Should the linear or nonlinear model be used to
% generate the actual output

%%%OUTPUTS%%%
% Y_pred: The output of the Smith's Predictor

%%% EQUATIONS OF MOTION %%%
%Steady-state values
%Obtained by solving Vss^2 = uss^2+vss^2+wss^2 and alpha_ss = arctan(wss/uss)
uss = VEq*cos(thetaEq); wss = VEq*sin(thetaEq); vss = 0;
%Others are 0
phiss = 0; thetass = thetaEq; psiss = 0; pss = 0; qss = 0; rss = 0;

%%% EOMS IN DISCRETE TIME %%%
%Initial states
Xss = [phiss; thetass; psiss; uss;vss;wss; pss;qss;rss];
Sund(:,1) = ICund(1:3); deltaXund(:,1) = ICund(4:12) - Xss;
Sdel(:,1) = ICdel(1:3,1); deltaXdel(:,1) = ICdel(4:12) - Xss;

deltaU = Input - [0;deEq;0];

i = in_delDT + out_delDT -1;

actual_index = 1; %Equivalent to l=k-d2; so the initial conditions start at l=k-d2 which is different from ode45 ICs

for k = 1:length(Time)-1
    if k-in_delDT-out_delDT-1>0
        %Delayed states one-step propagation
        deltaXdel(:,actual_index+1) = sysDT.A*deltaXdel(:,actual_index) + sysDT.B*deltaU(:, k-in_delDT-out_delDT-1);
        %Delayed output, y_hat_del
        Y_hat_del(:,actual_index) = eye(12)*[Sdel(:, actual_index); deltaXdel(:,actual_index)+Xss];
        
        %Calculating the next step positions using nonlinear equations and
        %delayed states
        phi = deltaXdel(1,actual_index+1) + Xss(1); theta = deltaXdel(2,actual_index+1) + Xss(2); psi = deltaXdel(3,actual_index+1) + Xss(3); 
        u = deltaXdel(4,actual_index+1) + Xss(4); v = deltaXdel(5,actual_index+1) + Xss(5); w = deltaXdel(6,actual_index+1) + Xss(6);
        RIB = expm(psi*hat(e3))*expm(theta*hat(e2))*expm(phi*hat(e1));
        Sdel(:, actual_index+1) = Sdel(:,actual_index) + RIB*[u;v;w]*Ts;

        %Undelayed states one-step propagation
        deltaXund(:,actual_index+1) = sysDT.A*deltaXund(:,actual_index) + sysDT.B*deltaU(:, k);
        %Undelayed output, y_hat_und
        Y_hat_und(:,actual_index) = eye(12)*[Sund(:, actual_index); deltaXund(:,actual_index)+Xss];
        
        %Calculating the next step positions using nonlinear equations and
        %delayed states
        phi_und = deltaXund(1,actual_index+1) + Xss(1); theta_und = deltaXund(2,actual_index+1) + Xss(2); psi_und = deltaXund(3,actual_index+1) + Xss(3); 
        u_und = deltaXund(4,actual_index+1) + Xss(4); v_und = deltaXund(5,actual_index+1) + Xss(5); w_und = deltaXund(6,actual_index+1) + Xss(6);
        RIB_und = expm(psi_und*hat(e3))*expm(theta_und*hat(e2))*expm(phi_und*hat(e1));
        Sund(:, actual_index+1) = Sund(:,actual_index) + RIB_und*[u_und;v_und;w_und]*Ts;
                
        %Calculate the predicted output
        Y_pred(:, actual_index) = Measurement(:,k) + Y_hat_und(:,actual_index) - Y_hat_del(:,actual_index); %Measurement at k is already delayed
        Time_pred(actual_index) = Time(k);
        actual_index = actual_index + 1;
    end
end


