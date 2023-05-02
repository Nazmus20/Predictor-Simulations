clear all; clc; close all;

%We found the frequency domain TF of the gimbal which needs to be
%converted to time domain for C++/ROS implementation. The conversion is
%done and then checked with the frequency domain response. The conversion
%formula: theta(s) = G(s)*U(s) --> theta(t) = L^{-1}{G(s)} U(t) = L^{-1} 
%{KG/s*exp^(-s*tau)} U(t) = KG*heaviside(t-tau)*integrate(U(t-tau)) = KG*Ts
%*heaviside(t-tau)*U(t-tau) 


s = tf('s');
KG = .98; tau = 1; Ts = .01; kd = tau/Ts;
del_act = .09; kdel_act = del_act/Ts;
G = tf(KG*exp(-s*del_act)/s); %Gimbal TF

t_vec = 0:Ts:10; %Time vector, s

%Creating the commanded input vector, deg/s
for i = 1:length(t_vec)
    if t_vec(i) > 3 && t_vec(i) <= 5 
        U(i) = 1;
    elseif t_vec(i) > 5 && t_vec(i) <= 7
        U(i) = -1;
    else
        U(i) = 0;
    end        
end

%Now taking the TF (frequency domain) approach, we obtain the estimated
%current and delayed outputs  
Out_curr = lsim(G, U, t_vec);
Out_del = lsim(G*exp(-s*tau), U, t_vec);
    
%The "actual output" (This should be the real-world measurements which are
%affected by noise and disturbances. But in this simulation they are
%assumed to be none. If the estimated model of the plant is accurate then
%the estimated delayed output and the actual delayed output are equal
Out_act = Out_del;

%Now taking the inverse laplace (time domain) approach
%Creating the estimated predicted output using Smith's predictor
th_pred_curr(1) = 0;
for i = 1:length(t_vec)
    if t_vec(i) - del_act <0
        th_hat_curr = 0; th_prev = 0;   
    else
        th_hat_curr = heaviside(t_vec(i) - del_act) * U(i-kdel_act);
    end
    
    th_pred_curr(i+1) = th_pred_curr(i) + KG*Ts*(th_hat_curr);
end

%Creating the estimated delayed output using Smith's predictor
th_pred_del(1) = 0;
for i = 1:length(t_vec)
    if t_vec(i) - del_act -tau <0
        th_hat_del = 0; th_prev = 0;   
    else
        th_hat_del = heaviside(t_vec(i) - del_act - tau) * U(i-kdel_act - kd);
    end
    
    th_pred_del(i+1) = th_pred_del(i) + KG*Ts*(th_hat_del);
end

%Smith's predictor formula: Predicted Output = Current delayed output +
%(Estimated predicted output - Estimated dlayed output)
for i = 1:length(t_vec)
    th_pred(i) = Out_act(i) + th_pred_curr(i) - th_pred_del(i);
end



theta(1) = 0; 
%Comparing with the previous method The previous formulation was used to
%collect human testing data. The formula was: theta_predicted =
%theta_predicted_previous + Ts*tau*input_command
for i =1:length(t_vec)-1
    if t_vec(i) - tau < 0
        theta(i+1) = theta(i); 
    else
        theta(i+1) = theta(i) + Ts*tau*U(i);
    end
end

%Plotting
plot(t_vec, U, 'k', t_vec, Out_act, 'k', t_vec, Out_curr, 'r-', t_vec, th_pred, 'b--')
hold on
xline(3, 'k--');
hold on
xline(5, 'k--');
xlabel('Time, $t~(s)$', 'Interpreter', 'Latex')
ylabel('Input, $\dot{\theta}~(deg/s)$ and output, $\theta~(deg)$', ...
    'Interpreter', 'Latex', 'FontSize', 12)

text(3,-.5, '\leftarrow t = 3 (s)')
text(5,-.5, '\leftarrow t = 5 (s)')
hold on
plot(t_vec,theta, 'g');
legend('Input', 'Actual output', 'Frequency domain output', ...
    'Time domain output', 'Previous output')
