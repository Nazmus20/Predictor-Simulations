function [pred_time, x_pred] = ...
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
V = 1*.0025*eye(12); %Constant if noise is stationary
%Initialize the noise covariance matrix R_vec(:,:,iter) = E[w(n)w'(n)]
W = 100000*eye(12);

%Delays used
Ts = Sim.Ts; %Sampling time
d1 = Sim.out_delDT; d2 = Sim.in_delDT; %Discretized delay
%figure
%hold on

z_hat_l(:,1) = available_states(:,1);
for k=1:length(Sim.t_vec)-1
    n = k-d1; l = k-d1-d2;
    if n>=1 && Sim.t_vec(k)<0
        x_hat_l_plus_one(:,n) = Sim.initial_state;
        P_l_plus_one(:,:,n) = eye(length(F));
        iter = 1;
    end
    if Sim.t_vec(k)>=0
        %One step predicted state
        x_hat_l(:,n) = F*x_hat_l_plus_one(:,n-1) + G*Sim.del_des_state(:,k-1);
        %State prediction error
        %x_tilde_l(:,iter+1) = available_states(:,k+1) - x_hat_l(:,iter+1);
        %State prediction covariance
        P_l(:,:,n) = F*P_l_plus_one(:,:,n-1)*F' + W;
        %One step predicted masurement
        z_hat_l(:,l) = H*x_hat_l(:,n);
        %Measurement prediction error (When C = I, measurement = state)
        z_tilde_l(:,l) = H*available_states(:,k) - z_hat_l(:,l);
        %Measurement prediction covariance
        S_l(:,:,n) = H*P_l(:,:,n)*H' + V;
        %Filter gain
        L(:,:,n) = P_l(:,:,n)*H'*inv(S_l(:,:,n));
        %Updated state estimate
        x_hat_l_plus_one(:,n) = x_hat_l(:,n)+L(:,:,n)*z_tilde_l(:,l);
        %Updated covariance matrix
        P_l_plus_one(:,:,n) = P_l(:,:,n) - L(:,:,n)*H*P_l(:,:,n);
        plot(Sim.t_vec(k), available_states(3,k), 'k+', Sim.t_vec(k), z_hat_l(3,l), 'ro')
        title('Plot of actual outputs vs estimated outputs')
        hold on
        mtx_sum=zeros(length(F),1); mtx_mul2 = F;
        
        for i=0:1:d1+d2
            mtx_mul = eye(size(F)); 
            for j=d1+d2:-1:i+1
                mtx_mul=mtx_mul*F;
            end
            mtx_mul2 = mtx_mul2*F;
            mtx_sum=mtx_sum+mtx_mul*G*Sim.des_state(:,l+i);
        end
        
        x_pred(:,iter) = F^(d1+d2+1)*(x_hat_l_plus_one(:,n)) + mtx_sum;
        pred_time(iter) = Sim.t_vec(k);
        iter = iter+1;
    end
end
xlabel('Time, t (s)')
ylabel('Height, z (m)') 
legend('Measured, $z_l$ (m)', 'Estimated, $\hat{z}_{l|l}$ (m)', 'Interpreter', 'latex', 'FontSize', 24)

figure
    plot(pred_time, z_tilde_l(3,:))
    xlabel('Time, t(s)')
    ylabel('Innovation, z_tilde')
    title('Plot of innovation')
    xlabel('Time, t (s)')
    ylabel('Height error, $\tilde{z}$ (m)', 'Interpreter', 'latex')
end


