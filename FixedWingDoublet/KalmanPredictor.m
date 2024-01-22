function [Time_pred, Y_pred, P] = ...
    KalmanPredictor(sysDT, Time, Measurement, Input, VEq, thetaEq, deEq, IC, in_delDT, out_delDT, e1, e2, e3, Ts)

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
Shat(:,1) = IC(1:3); deltaXhatPlus(:,1) = IC(4:12) - Xss;
Pplus(:,:,1) = eye(9);

NF=1;
phi_noise_std = NF*.01; theta_noise_std = NF*.01; psi_noise_std = NF*.01; 
u_noise_std = NF*.1; v_noise_std = NF*.1; w_noise_std = NF*.1; 
p_noise_std = NF*.01; q_noise_std = NF*.01; r_noise_std = NF*.01;

W = 0.01*eye(9); V = diag([phi_noise_std^2;theta_noise_std^2;psi_noise_std^2;u_noise_std^2; v_noise_std^2;w_noise_std^2;p_noise_std^2;q_noise_std^2;r_noise_std^2]);

deltaU = Input - [0;deEq;0];

%Resize the measurement
delta_measurement = Measurement(4:12,:)-sysDT.C*Xss;

i = in_delDT+out_delDT - 1 ;

predictor_index = 1; %Equivalent to l=k-d2; so the initial conditions start at l=k-d2 which is different from ode45 ICs

for k = 1:length(Time)
    if k-in_delDT-out_delDT-1>0
        %One step prediction
        deltaXhatMinus(:,predictor_index) = sysDT.A*deltaXhatPlus(:,predictor_index) + sysDT.B*deltaU(:, k-in_delDT-out_delDT-1);
        Pminus(:,:,predictor_index) = sysDT.A*Pplus(:,:,predictor_index)*sysDT.A' + W;
        
        deltaYhat(:,predictor_index) = sysDT.C*deltaXhatMinus(:,predictor_index); %Measurement estimate
        deltaYtilde(:,predictor_index) = delta_measurement(:,k) - deltaYhat(:,predictor_index); %Innovation
        
        L(:,:,predictor_index) = Pminus(:,:,predictor_index)*sysDT.C'*inv(sysDT.C*Pminus(:,:,predictor_index)*sysDT.C'+V); %Kalman gain
        
        %Measurement updated estimate
        deltaXhatPlus(:,predictor_index+1) = deltaXhatMinus(:,predictor_index)+L(:,:,predictor_index)*deltaYtilde(:,predictor_index);
        Pplus(:,:,predictor_index+1) = Pminus(:,:,predictor_index) - L(:,:,predictor_index)*sysDT.C*Pminus(:,:,predictor_index);
        
        %Calculating the positions using nonlinear equations
        phi = deltaXhatPlus(1,predictor_index)+Xss(1); theta = deltaXhatPlus(2,predictor_index)+Xss(2);
        psi = deltaXhatPlus(3,predictor_index)+Xss(3); u = deltaXhatPlus(4,predictor_index)+Xss(4);
        v = deltaXhatPlus(5,predictor_index)+Xss(5); w = deltaXhatPlus(6,predictor_index)+Xss(6); 
        RIB = expm(psi*hat(e3))*expm(theta*hat(e2))*expm(phi*hat(e1));
        
        %Update the position using nolinear equations
        Shat(:,predictor_index+1) = Shat(:,predictor_index) + RIB*[u;v;w]*Ts;
        
        %Output prediction over delay, y_hat_pred by looping
        %deltaXhatPlus(:,actual_index+1) and Pplus(:, actual_index+1) "in_delDT+out_delDT" times
        deltaXhat_prev = deltaXhatPlus(:,predictor_index+1);
                    
        P_prev = Pplus(:,:,predictor_index+1); S_prev = Shat(:,predictor_index+1); RIB_prev=RIB; u_prev=u; v_prev=v; w_prev=w;
        
        for j = 0:i
            deltaXhat_pred = sysDT.A*deltaXhat_prev + sysDT.B*deltaU(:, k-in_delDT-out_delDT+j);
            P_pred = sysDT.A*P_prev*sysDT.A' + W;
            %Calculating the positions using nonlinear equations
            phi_pred = deltaXhat_pred(1)+Xss(1); theta_pred = deltaXhat_pred(2)+Xss(2); 
            psi_pred = deltaXhat_pred(3)+Xss(3); u_pred = deltaXhat_pred(4)+Xss(4);
            v_pred = deltaXhat_pred(5)+Xss(5); w_pred = deltaXhat_pred(6)+Xss(6);
            
            RIB_pred = expm(psi_pred*hat(e3))*expm(theta_pred*hat(e2))*expm(phi_pred*hat(e1));
            
            S_pred = S_prev + RIB_prev*[u_prev;v_prev;w_prev]*Ts;
            
            deltaXhat_prev = deltaXhat_pred; S_prev = S_pred; P_prev = P_pred; RIB_prev=RIB_pred; u_prev=u_pred; v_prev=v_pred; w_prev=w_pred; 
        end
        %Sstar=Measurement(1:3,k) + S_pred - Shat(:,predictor_index+1);
        Sstar=S_pred;
        %Calculate the predicted output
        Y_pred(:, predictor_index) = eye(12)*[Sstar; deltaXhat_pred+Xss];
        Y_pred(1, predictor_index)=Y_pred(1, predictor_index)-0;
        Time_pred(predictor_index) = Time(k);
        P(:,:,predictor_index) = P_pred;
        predictor_index = predictor_index + 1;
    end        
end



