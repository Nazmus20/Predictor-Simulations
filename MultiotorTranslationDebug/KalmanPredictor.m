function [Time, State, L, Cov] = ...
    KalmanPredictor(sysDT, Sim, Y_l)

% KalmanPredictor.m
%
% Kalman Predictor simulation which uses the exact Bar-Shalom formulas

%%%INPUTS%%%
% sysDT: The discrete time (DT) system of the UAV
% t_vec: Time_vector of the simulation, s
% x_vec0: Initial states for the simulation to start. The UAV is assumed to
% start from its steady-states
% Klqr: The LQR gain matrix in CT
% ref_vec: The actual reference vector without any delay
% in_delDT: The value of the delay in the outgoing signal from the
% groundstation to the UAV, seconds
% out_delDT: The value of the delay in the incoming signal from the UAV to
% the groundstation, seconds
% z_vec_linear: This is the roundtrip delay between the groundstation-UAV-
% groundstation. Here the output are the states found by using a linear
% dynamic model. Possible only when y = C*x = x @ C = I. 
% z_vec_nonlinear: This is the roundtrip delay between the groundstation-
% UAV-groundstation. Here the output are the states found by using a
% nonlinear dynamic model. Possible only when y = C*x = x @ C = I. 

%%%OUTPUTS%%%
% predicted_states_linear: The output of the Kalman Predictor (if C = I
% then they are also the states). Outputs are obtained when a linear
% dynamic model is used x(k+1) = A*x(k) + B*u(k) + v(k); z(k)=C*x(k) + w(k)
% predicted_states_nonlinear: The output of the Kalman Predictor (if C = I
% then they are also the states). Outputs are obtained when a nonlinear
% dynamic model is used x(k+1) = f(x(k), u(k), v(k)); y=g(x(k), u(k), w(k))

%State/Output matrices
F = sysDT.A; G =sysDT.B; H = sysDT.C;

%Initialize the disturbance covariance matrix V = E[v(n)v'(n)]
V = 1*.0025*eye(12); %Constant if noise is stationary
%Initialize the noise covariance matrix W = E[w(n)w'(n)]
W = .0001*eye(12);

%Delays used
Ts = Sim.Ts; %Sampling time
d1 = Sim.out_delDT; d2 = Sim.in_delDT; %Discretized delay

figure(4)
hold on

i = d1+d2+1 % Prediction horizon

% Initial value
P_prev(:,:,1) = F*eye(size(F))*F'+W; %Initial covariance
x_hat_prev(:,1) = Sim.initial_state;
L = eye(size(F)); % Kalman gain
for idx = 2:1:i
    x_hat_prev(:,idx) = Sim.initial_state;
    P_prev(:,:,idx) = F*P_prev(:,:,idx-1)*F'+W;
end
iter = 1;
for k=1:length(Sim.t_vec) %k is the current time step  
    n = k-d1; l = k-d2;
    % Waiting until the delay horizon is passed and the input buffer is filled
    if l>0 && l-d1>0
        L = P_prev(:,:,1)*H'*inv(H*P_prev(:,:,1)*H' + V);

        % Delayed inputs and states
        y_available = Y_l(:,k); 
        y_tilde = y_available-H*x_hat_prev(:,1); % Innovation
        mtx_sum1 = zeros(length(F),1);
        for m = 0:1:i-1
            mtx_mul1 = eye(size(F));
            for j = i-1:-1:m+1
                mtx_mul1 = mtx_mul1*F;
            end
            mtx_sum1 = mtx_sum1 + mtx_mul1*G*Sim.des_state(:,l-d1+m);
        end

        for idx = 1:1:i
            x_hat_pred(:,idx) = F*x_hat_prev(:,idx) + (F^i)*L*y_tilde ;%+ mtx_sum1;
            P_pred(:,:,idx) = F*P_prev(:,:,idx)*F'+W;
            time_pred(idx) = Sim.t_vec(k+idx-1);
        end
        
        %Predicted states and their covariances for d1+d2+1 steps
        Time(iter) = Sim.t_vec(k);
        State(:,iter) = x_hat_pred(:,end);
        Cov(:,:,iter) = P_pred(:,:,end);

        plot(time_pred, x_hat_pred(2,:), 'ko')
        xlabel('Time, t(s)')
        ylabel('Innovation, y_tilde')
        title('Plot of innovation')
        hold on 

        % Prediction update
        x_hat_prev = x_hat_pred;
        P_prev = P_pred;
        iter = iter+1;
    end
end
end