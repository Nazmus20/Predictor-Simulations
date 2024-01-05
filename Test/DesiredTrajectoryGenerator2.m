function [State, Input, Time] = DesiredTrajectoryGenerator2(A)

Ts = .0001;
t_vec = 0:Ts:20; %High precision time vector

phiD = 3.52877*pi/180; thetaD = 0; psiD = 0; uD = 0; vD = 2.12612; 
wD = -0.131111; pD = 0; qD = 0; rD = 0; 

u_TD = -14.7988; u_phiD = 0; u_psiD = 0; u_thetaD = 0;

%Initial error states, initially the UAV is at hover. So error is from
%zeros to the desired states; except z state which is at 5 m.

State(:,1) = [0; 0; -5; phiD; thetaD; psiD; uD; vD; wD; pD; qD; rD];
State_dot(:,1) = [0; 2.13015; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
Input(:,1) = [u_TD; u_phiD; u_thetaD; u_psiD];

for i=2:length(t_vec)
    State(:,i) = State(:,1)+State_dot*t_vec(i);
    Input(:,i) = Input(:, i-1);
end

Time = t_vec;