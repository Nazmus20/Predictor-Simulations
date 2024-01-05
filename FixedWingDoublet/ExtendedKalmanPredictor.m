function [Time_pred, Y_pred, P] = ...
    ExtendedKalmanPredictor(Time, Measurement, Input, IC, in_delDT, out_delDT, Ts)

global e1 e2 e3 m rho g Inertia b c S ...
    Cj2 Cj Cj0 ...
    Cxalpha Cxde Cx0 Cxq Cxalpha2...
    Czalpha Czq Cz0 Czde...
    Cmalpha Cmq Cmde Cm0 ...
    Cybeta Cyp Cyr Cyda Cydr Cy0 ...
    Clbeta Clp Clda Clr Cldr Cl0...
    Cnbeta Cnr Cnda Cndr Cnp Cn0...
    drpsEq PropDia nProp etaProp VEq deEq thetaEq

i = in_delDT+out_delDT-1 ; %Delay horizon

%Noise
NF=.5;
phi_noise_var = NF*.01; theta_noise_var = NF*.01; psi_noise_var = NF*.01; 
u_noise_var = NF*.1; v_noise_var = NF*.1; w_noise_var = NF*.1; 
p_noise_var = NF*.01; q_noise_var = NF*.01; r_noise_var = NF*.01;

W = 1*eye(9); V = diag([phi_noise_var;theta_noise_var;psi_noise_var;u_noise_var; v_noise_var;w_noise_var;p_noise_var;q_noise_var;r_noise_var]);

predictor_index = 1; %Equivalent to l=k-d2; so the initial conditions start at l=k-d2 which is different from ode45 ICs

%Initial state and covariance estimates
Pplus(:,:,1) = eye(9);
xhatplus = IC;

for k = 1:length(Time)
    if k-in_delDT-out_delDT-1>0
        xhatminus = xhatplus + NonlinearFixedWingEOM(Time(k-out_delDT-1),xhatplus,Input(k-in_delDT-out_delDT-1))*Ts;
        
        Pminus = Pplus + 
        
        
        
        %One step prediction
        deltaXhatMinus(:,predictor_index) = sysDT.A*deltaXhatPlus(:,predictor_index) + sysDT.B*Input(:, k-in_delDT-out_delDT-1);
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
                    
        P_prev = Pplus(:,:,predictor_index+1); S_prev = Shat(:,predictor_index+1);
        
        for j = 0:i
            deltaXhat_pred = sysDT.A*deltaXhat_prev + sysDT.B*deltaU(:, k-in_delDT-out_delDT+j);
            P_pred = sysDT.A*P_prev*sysDT.A' + W;
            %Calculating the positions using nonlinear equations
            phi_pred = deltaXhat_pred(1)+Xss(1); theta_pred = deltaXhat_pred(2)+Xss(2); 
            psi_pred = deltaXhat_pred(3)+Xss(3); u_pred = deltaXhat_pred(4)+Xss(4);
            v_pred = deltaXhat_pred(5)+Xss(5); w_pred = deltaXhat_pred(6)+Xss(6);
            
            RIB_pred = expm(psi_pred*hat(e3))*expm(theta_pred*hat(e2))*expm(phi_pred*hat(e1));
            
            S_pred = S_prev + RIB_pred*[u_pred;v_pred;w_pred]*Ts;
            
            deltaXhat_prev = deltaXhat_pred; S_prev = S_pred; P_prev = P_pred;
        end
     
        %Calculate the predicted output
        Y_pred(:, predictor_index) = eye(12)*[S_pred; deltaXhat_pred+Xss];
        Time_pred(predictor_index) = Time(k);
        P(:,:,predictor_index) = P_pred;
        predictor_index = predictor_index + 1;
    end        
end

figure
plot(Time_pred, Y_pred(1,:), 'k-', Time, Measurement(1,:), 'r--')
%plot(Time_pred, deltaYhat(2,:), 'k-', Time, delta_measurement(2,:), 'r--', Time_pred, Y_pred(5,:), 'b--')
title('Y_pred and measurement')



























































