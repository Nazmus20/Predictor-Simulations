function [x_pred] = ...
    KalmanPredictor2(sysDT, Sim, available_states)

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

%Initial state estimate and output estimate
x_hat_l(:,1) = Sim.initial_state; z_hat_l(:,1) = H*x_hat_l(:,1);

%Initial state estimate error
x_tilde_l(:,1) = available_states(:,1) - x_hat_l(:,1);
%Initial measurement prediction error
z_tilde_l(:,1) = H*x_tilde_l(:,1);
%Initial covariance estimate
P_l(:,:,1) = eye(12);
%Initial filter gain
%W(:,:,1) = zeros(12);
%Initial updated state estimate
x_hat_l_plus_one(:,1) = Sim.initial_state;
%Initial updated state covariance
P_l_plus_one(:,:,1) = eye(length(Sim.initial_state));
%Initial measurememnt prediction covariance
S_l_plus_one(:,:,1) = eye(length(Sim.initial_state));
%Initialize the disturbance covariance matrix Q_vec(:,:,iter) = E[v(n)v'(n)]
V = .01*eye(12); %Constant if noise is stationary
%Initialize the noise covariance matrix R_vec(:,:,iter) = E[w(n)w'(n)]
W = .01*eye(12);

%Delays used
Ts = Sim.Ts; %Sampling time
d1 = Sim.out_delDT; d2 = Sim.in_delDT; %Discretized delay
%figure
%hold on
%Obtain the array of inputs
for k=1:length(Sim.t_vec)-1
    %One step predicted state
    x_hat_l(:,k+1) = F*x_hat_l_plus_one(:,k) + G*Sim.del_des_state(:,k);
    %State prediction error
    x_tilde_l(:,k+1) = available_states(:,k+1) - x_hat_l(:,k+1);
    %State prediction covariance
    P_l(:,:,k+1) = F*P_l_plus_one(:,:,k)*F' + W;
    %One step predicted masurement
    z_hat_l(:,k+1) = H*x_hat_l(:,k+1);
    %Measurement prediction error (When C = I, measurement = state)
    z_tilde_l(:,k+1) = available_states(:,k+1) - z_hat_l(:,k+1);
    %Measurement prediction covariance
    S_l(:,:,k+1) = H*P_l(:,:,k+1)*H' + V;
    %Filter gain
    L(:,:,k+1) = P_l(:,:,k+1)*H'*inv(S_l(:,:,k+1));
    %Updated state estimate
    x_hat_l_plus_one(:,k+1) = x_hat_l(:,k+1)+L(:,:,k+1)*z_tilde_l(:,k+1);
    %Updated covariance matrix
    P_l_plus_one(:,:,k+1) = P_l(:,:,k+1) - L(:,:,k+1)*H*P_l(:,:,k+1);
    x_pred(:,k) = Sim.initial_state;
    if Sim.t_vec(k)>=0
        n = k-d1;        
        mtx_sum=zeros(length(F),1); 
        for i=0:1:d1-1
            mtx_mul = eye(size(F));
            for j=d1-1:-1:i+1
                mtx_mul=mtx_mul*F;
            end
            mtx_sum=mtx_sum+mtx_mul*G*Sim.des_state(:,n+i);
        end
        x_pred(:,k) = F^(d1)*(F*x_hat_l_plus_one(:,k-1) + L(:,:,k)*z_tilde_l(:,k)) + mtx_sum;
        pred_time(k) = Sim.t_vec(k);
    end  
end

end

