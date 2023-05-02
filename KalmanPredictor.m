function [y_pred] = ...
    KalmanPredictor(sysDT, t_vec, x_vec0, ref_vec, in_del, out_del, ...
    x_vec_linear, x_vec_nonlinear, z_vec_linear, z_vec_nonlinear, Klqr, ...
    uSS)

% KalmanPredictor.m
%
% Kalman Predictor simulation

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
x_hat_l(:,1) = x_vec0; z_hat_l(:,1) = H*z_vec_linear(:,1);

%Start with an incorrect state !!!!!!!!!!!!!
x_hat_l(:,1) = zeros(12,1);

%Initial state estimate error
x_tilde_l(:,1) = x_vec_linear(:,1) - x_hat_l(:,1);
%Initial measurement prediction error
z_tilde_l(:,1) = H*x_vec_linear(:,1) - z_hat_l(:,1);
%Initial covariance estimate
P_l(:,:,1) = eye(12);
%Initial filter gain
W(:,:,1) = zeros(12);
%Initial updated state estimate
x_hat_l_plus_one(:,1) = zeros(length(x_vec0),1);
%Initial updated state covariance
P_l_plus_one(:,:,1) = zeros(length(x_vec0));
%Initial measurememnt prediction covariance
S_l_plus_one(:,:,1) = eye(length(x_vec0));

%Initialize the disturbance covariance matrix Q_vec(:,:,iter) = E[v(n)v'(n)]
Q_vec = eye(12); %Constant if noise is stationary
%Initialize the noise covariance matrix R_vec(:,:,iter) = E[w(n)w'(n)]
R_vec = eye(12);

%Delays used
Ts = sysDT.Ts; %Sampling time
d1 = round(out_del/Ts); d2 = round(in_del/Ts); %Discretization of the delay

%Add delay to the reference vector (outgoing delay) once
del_ref_vec = addDelay(t_vec, ref_vec, out_del, x_vec0);

for n = 1:length(t_vec)-d1-1    
%Compute the actual LQR input to the plant
ulqr(:,n) = - Klqr * (x_vec_linear(:,n) - del_ref_vec(:,n));
%One step predicted state
x_hat_l(:,n+1) = F*(x_hat_l(:,n)) + G*ulqr(:,n);
%State prediction error
x_tilde_l(:,n+1) = x_vec_linear(:,n+1) - x_hat_l(:,n+1);
%State prediction covariance
P_l(:,:,n+1) = F*P_l(:,:,n)*F' + Q_vec;
%One step predicted masurement
z_hat_l(:,n+1) = H*x_hat_l(:,n+1);
%Measurement prediction error (When C = I, measurement = state)
z_tilde_l(:,n+1) = H*x_vec_linear(:,n+1) - z_hat_l(:,n+1);
%Measurement prediction covariance
S_l_plus_one(:,:,n+1) = H*P_l(:,:,n+1)*H' + R_vec;
%Filter gain
W(:,:,n+1) = P_l(:,:,n+1)*H'*inv(S_l_plus_one(:,:,n+1));
%Updated state estimate
x_hat_l_plus_one(:,n+1) = x_hat_l(:,n+1) + W(:,:,n+1)*...
    z_tilde_l(:,n+1);
%Updated covariance matrix
P_l_plus_one(:,:,n+1) = P_l(:,:,n+1) - W(:,:,n+1)*...
    S_l_plus_one(:,:, n+1)*W(:,:,n+1)';

A = eye(12)-W(:,:,n+1)*H; 
%Predict the states and output
%sum = zeros(length(x_vec0));
Mat =[]; REF = [];
for i = 1:d1
    Mat = [Mat, A*F^(d1-i)*G*(-Klqr)];
    REF = [REF;H*x_vec_linear(:,n) - ref_vec(:,n+i-1)];
end
x_pred(:,n) = A*F^d1*(x_hat_l(:,n)) + Mat*REF + W(:,:,n+1)*H*...
    x_vec_linear(:,n+1);
y_pred(:,n) = H*x_pred(:,n);
end

