function [Time_pred, Y_pred] = ...
    SmithPredictor2(sysDT, Time, Measurement, Input, VEq, thetaEq, deEq, IC, in_delDT, out_delDT, e1, e2, e3, Ts)

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
S(:,1) = IC(1:3); deltaX(:,1) = IC(4:12) - Xss;
S_prev = S(:,1);
deltaU = Input - [0;deEq;0];

i = in_delDT + out_delDT -1;

actual_index = 1; %Equivalent to l=k-d2; so the initial conditions start at l=k-d2 which is different from ode45 ICs
for k = 1:length(Time)-1
    if k-in_delDT-out_delDT-1>0
        %Output with delay, Y_hat_l
        Y_hat_del(:,actual_index) = eye(12)*[S(:, actual_index); deltaX(:,actual_index)+Xss];
        
        %Simulation of next step using linear system with delay in input
        deltaX(:,actual_index+1) = sysDT.A*deltaX(:,actual_index) + sysDT.B*deltaU(:, k-in_delDT-out_delDT-1);
        %Calculating the positions using nonlinear equations
        phi = deltaX(1,actual_index) + Xss(1); theta = deltaX(2,actual_index) + Xss(2); psi = deltaX(3,actual_index) + Xss(3); 
        u = deltaX(4,actual_index) + Xss(4); v = deltaX(5,actual_index) + Xss(5); w = deltaX(6,actual_index) + Xss(6);
        RIB = expm(psi*hat(e3))*expm(theta*hat(e2))*expm(phi*hat(e1));
        S(:, actual_index+1) = S(:,actual_index) + RIB*[u;v;w]*Ts;

        %Output prediction over delay, y_hat_pred by looping
        %deltaX(:,actual_index+1) and S(:, actual_index+1) "in_delDT+out_delDT" times
        deltaX_prev = deltaX(:,actual_index+1);
        S_prev = S(:,actual_index+1);
        for j = 0:i       
            phi_prev = deltaX_prev(1) + Xss(1); theta_prev = deltaX_prev(2) + Xss(2); psi_prev = deltaX_prev(3) + Xss(3); 
            u_prev = deltaX_prev(4) + Xss(4); v_prev = deltaX_prev(5) + Xss(5); w_prev = deltaX_prev(6) + Xss(6);
            RIB_prev = expm(psi_prev*hat(e3))*expm(theta_prev*hat(e2))*expm(phi_prev*hat(e1));

            %Calculating the linear states
            deltaX_pred = sysDT.A*deltaX_prev + sysDT.B*deltaU(:, k-in_delDT-out_delDT+j);
            phi_pred = deltaX_pred(1)+Xss(1); theta_pred = deltaX_pred(2)+Xss(2); 
            psi_pred = deltaX_pred(3)+Xss(3); u_pred = deltaX_pred(4)+Xss(4);
            v_pred = deltaX_pred(5)+Xss(5); w_pred = deltaX_pred(6)+Xss(6);
            
            S_pred = S_prev + RIB_prev*[u_prev;v_prev;w_prev]*Ts;
            
            deltaX_prev = deltaX_pred; S_prev = S_pred;   
        end
     
        %Calculate the predicted output
        Y_hat_pred(:, actual_index) = eye(12)*[S_pred; deltaX_pred+Xss];
        Y_pred(:, actual_index) = Measurement(:,k) + Y_hat_pred(:,actual_index) - Y_hat_del(:,actual_index); %Measurement at k is already delayed
        Time_pred(actual_index) = Time(k);
        actual_index = actual_index + 1;
    end
end

% figure
% plot(Time_pred, Y_hat_pred(1,:), 'k-', Time_pred, Y_hat_del(1,:), 'b--', Time, Measurement(1,:), 'r--')
% title('SP')