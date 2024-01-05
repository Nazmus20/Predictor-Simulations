function [Time, State, L, y_tilde] = ...
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

i = d1+d2+1; % Prediction horizon

% Initial value
P_plus = eye(size(F)); %Initial covariance
x_hat_plus = Sim.initial_state;

iter = 1;
for k=1:length(Sim.t_vec)-1 %k is the current time step  
    n = k-d1; l = k-d2;
    % Waiting until the delay horizon is passed and the input buffer is filled
    if l>0 && l-d1>0
        x_hat_minus = F*x_hat_plus + G*Sim.des_state(:,l-d1)
        P_minus = F*P_plus*F' + W;
        
        L = P_minus*H'*inv(H*P_minus*H' + V);

        y_available = Y_l(:,k+1);
        y_tilde = y_available-H*x_hat_minus % Innovation
        
        mtx_sum1 = zeros(length(F),1);
        for m = 0:1:i-1
            mtx_mul1 = eye(size(F));
            for j = i-1:-1:m+1
                mtx_mul1 = mtx_mul1*F;
            end
            mtx_sum1 = mtx_sum1 + mtx_mul1*G*Sim.des_state(:,l-d1+m);
        end
        
        x_hat_pred = F^(i)*x_hat_plus + mtx_sum1 + F^(i-1)*L*y_tilde 
        P_pred(:,:,1) = P_plus;
        for idx = 2:1:i
            P_pred(:,:,idx) = F*P_pred(:,:,idx-1)*F'+W;
        end
        
        x_hat_plus = x_hat_minus + L*y_tilde
        P_plus = P_minus - L*H*P_minus
        
        %Predicted states and their covariances for d1+d2+1 steps
        Time(iter) = Sim.t_vec(k);
        State(:,iter) = x_hat_pred;
        Cov(:,:,iter) = P_pred(:,:,end);

        plot(Time(iter), x_hat_pred(8), 'ko', Time(iter), Sim.des_state(8, l-d1+1), 'ro')
        xlabel('Time, t(s)')
        ylabel('Innovation, y_tilde')
        title('Plot of innovation')
        hold on 

        % Prediction update
        iter = iter+1;
    end
end
end