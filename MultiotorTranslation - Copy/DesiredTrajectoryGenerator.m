function [ref_states, ref_state_derivs, ref_inputs] = ...
    DesiredTrajectoryGenerator(fxu, Sim, Values, Param, des_vals)

% DesiredTrajectoryGenerator.m
%
% Desired trajectory generator for the LQR/PID or other controller to
% follow. Solve the nonlinear EOM to get the relationship and set some
% desired thrust to obtain the desired values of the other inputs and
% states. In total there are 12-states: [x_D; y_D; z_D; phi_D; theta_D; 
% psi_D; u_D; v_D; w_D; p_D; q_D; r_D], and 4-inputs: [u_T_D; u_phi_D; 
% u_theta_D; u_psi_D]

%%%INPUTS%%%
% fxu: The 12 linear symbolic EOMs, obtained from MultirotorEOM()
% uSS: Numerical values of the input at the steady-state
% Klqr: The LQR gain matrix obtained from the linearized EOMs, applied to
% control a linear/nonlinear system
% t_vec: Time vector at which points the values are needed to be found

%%%OUTPUTS%%%
% dx: State derivatives of the 12-states at each t_vec step
% ucntrl: The control input of the controller to the UAV at each t_vec step

syms  xD yD zD phiD thetaD psiD uD vD wD pD qD rD u_TD u_phiD u_thetaD ...
    u_psiD x_dotD y_dotD z_dotD phi_dotD theta_dotD psi_dotD u_dotD ...
    v_dotD w_dotD p_dotD q_dotD r_dotD real


x(1)=0; y(1) = 0;  z(1) = -5; phi(1) = 1*pi/180; theta(1) = 0; psi(1) = 0; 
u(1) = 0; v(1) = 1.133313; w(1) = -.019779; p(1) = 0; q(1) = 0; r(1) = 0;
u_T(1) = -14.7127; u_phi(1) = 0; u_theta(1) = 0; u_psi(1) = 0;
xdot(1) = 0; ydot(1) = 1.1333; zdot(1) = 0; phi_dot(1) = 0; 
theta_dot(1) = 0; psi_dot(1) = 0; u_dot(1) = 0; v_dot(1) = 0;
w_dot(1) = 0; p_dot(1) = 0; q_dot(1) = 0; r_dot(1) = 0;
   
%Create the reference trajectories
for i=2:length(Sim.t_vec)
    phi(i) = 1*pi/180;
    theta(i) = 0;
    psi(i) = 0;
    u(i) = 0;
    v(i) = 1.13313;%2.52759;
    w(i) = -0.0197789;%-.221136;
    p(i) = 0;
    q(i) = 0;
    r(i) = 0;
    u_T(i) = -14.7217;%-14.8834; 
    u_phi(i) = 0; u_theta(i) = 0; u_psi(i) = 0;
    xdot(i) = 0; ydot(i) = 1.1333;%2.53724; 
    zdot(i) = 0; phi_dot(i) = 0; 
    theta_dot(i) = 0; psi_dot(i) = 0; u_dot(i) = 0; v_dot(i) = 0;
    w_dot(i) = 0; p_dot(i) = 0; q_dot(i) = 0; r_dot(i) = 0;
    x(i) = x(i-1)+xdot(i)*(Sim.t_vec(i)-Sim.t_vec(i-1));
    y(i) = y(i-1)+ydot(i)*(Sim.t_vec(i)-Sim.t_vec(i-1));
    z(i) = -5;
end
ref_states = [x;y;z;phi;theta;psi;u;v;w;p;q;r];
ref_state_derivs = [xdot; ydot; zdot; phi_dot; theta_dot; psi_dot; u_dot; v_dot; w_dot; p_dot; q_dot; r_dot];
ref_inputs = [u_T; u_phi; u_theta; u_psi];

