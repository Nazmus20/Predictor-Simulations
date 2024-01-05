function [dx, ulqr] = NonlinearMultirotorTrajectory(tdum, xdum, fxu, ...
    Klqr, Values, Sim)

% NonlinearMultirotorTrajectory.m
%
% Nonlinear multirotor trajectory at some specified values.
% Initially starts from steady-state/hover. If an LQR controller is used 
% then the input is ue=uref - Klqr*(x - xref) which replaces the inputs 
% u in f(x,u) The state xe=x-x_ref is used in place of x in f(x,u)

%%%INPUTS%%%
% fxu: The 12 non-linear symbolic EOMs, obtained from LinearMultirotor()
% uSS: Numerical values of the input at the steady-state
% Klqr: The LQR gain matrix obtained from the linearized EOMs, applied to
% control a nonlinear system
% x_ref: The reference vector
% t_vec: Time vector at which points the values are needed to be found
% tdum: ode45() evaluation time steps
% xdum: ode45() evaluated solutions 

%%%OUTPUTS%%%
% dx: State derivatives of the 12-states at each t_vec step
% ucntrl: The control input of the controller to the UAV at each t_vec step

%Linear interpolation of the desired states, derivatives, and inputs at
%each ode45 time step, t
xref = interp1(Sim.t_vec, Sim.del_des_state(1,:), tdum);
yref = interp1(Sim.t_vec, Sim.del_des_state(2,:), tdum);
zref = interp1(Sim.t_vec, Sim.del_des_state(3,:), tdum);
phiref = interp1(Sim.t_vec, Sim.del_des_state(4,:), tdum);
thetaref = interp1(Sim.t_vec, Sim.del_des_state(5,:), tdum);
psiref = interp1(Sim.t_vec, Sim.del_des_state(6,:), tdum);
uref = interp1(Sim.t_vec, Sim.del_des_state(7,:), tdum);
vref = interp1(Sim.t_vec, Sim.del_des_state(8,:), tdum);
wref = interp1(Sim.t_vec, Sim.del_des_state(9,:), tdum);
pref = interp1(Sim.t_vec, Sim.del_des_state(10,:), tdum);
qref = interp1(Sim.t_vec, Sim.del_des_state(11,:), tdum);
rref = interp1(Sim.t_vec, Sim.del_des_state(12,:), tdum);
des_state = [xref; yref; zref; phiref; thetaref; psiref; uref; vref; ...
    wref; pref; qref; rref];

xref_dot = interp1(Sim.t_vec, Sim.del_des_state_deriv(1,:), tdum);
yref_dot = interp1(Sim.t_vec, Sim.del_des_state_deriv(2,:), tdum);
zref_dot = interp1(Sim.t_vec, Sim.del_des_state_deriv(3,:), tdum);
phiref_dot = interp1(Sim.t_vec, Sim.del_des_state_deriv(4,:), tdum);
thetaref_dot = interp1(Sim.t_vec, Sim.del_des_state_deriv(5,:), tdum);
psiref_dot = interp1(Sim.t_vec, Sim.del_des_state_deriv(6,:), tdum);
uref_dot = interp1(Sim.t_vec, Sim.del_des_state_deriv(7,:), tdum);
vref_dot = interp1(Sim.t_vec, Sim.del_des_state_deriv(8,:), tdum);
wref_dot = interp1(Sim.t_vec, Sim.del_des_state_deriv(9,:), tdum);
pref_dot = interp1(Sim.t_vec, Sim.del_des_state_deriv(10,:), tdum);
qref_dot = interp1(Sim.t_vec, Sim.del_des_state_deriv(11,:), tdum);
rref_dot = interp1(Sim.t_vec, Sim.del_des_state_deriv(12,:), tdum);
des_state_deriv = [xref_dot; yref_dot; zref_dot; phiref_dot; ...
    thetaref_dot; psiref_dot; uref_dot; vref_dot; wref_dot; ...
    pref_dot; qref_dot; rref_dot];

u_Tref = interp1(Sim.t_vec, Sim.del_des_input(1,:), tdum);
u_phiref = interp1(Sim.t_vec, Sim.del_des_input(2,:), tdum);
u_thetaref = interp1(Sim.t_vec, Sim.del_des_input(3,:), tdum);
u_psiref = interp1(Sim.t_vec, Sim.del_des_input(4,:), tdum);
des_input = [u_Tref; u_phiref; u_thetaref; u_psiref];

%Compute the LQR input to the plant (From Eqn B.9)
ucntrl = des_input - Klqr*(xdum - des_state);

%Using symbolic functions are inefficient, so converting them to MATLAB
% %function handle and computing the states
x = xdum(1); y = xdum(2); z = xdum(3); phi_r = xdum(4); theta_r = xdum(5);
psi_r= xdum(6); u = xdum(7); v = xdum(8); w = xdum(9); p = xdum(10);
q = xdum(11); r = xdum(12);

m = Values.m; g = Values.g; 
Ixx = Values.I(1,1); Iyy = Values.I(2,2); Izz = Values.I(3,3); 
Ixy = Values.I(1,2); Iyz = Values.I(2,3); Izx = Values.I(1,3);
Dx = Values.aero(1); Dy = Values.aero(2); Dz = Values.aero(3);
rho = Values.rho;
u_T = ucntrl(1); u_phi = ucntrl(2); 
u_theta = ucntrl(3); u_psi = ucntrl(4);

%fxu@(Dx,Dy,Dz,Ixx,Ixy,Iyy,Iyz,Izx,Izz,g,m,p,phi_r,psi_r,q,r,rho,theta_r,
%u,u_T,u_phi,u_psi,u_theta,v,w) replace the values properly. Nonlinear 
%model: x_dot = f(x-x_ref,u-u_ref)
x_dot = feval(fxu, Dx,Dy,Dz,Ixx,Ixy,Iyy,Iyz,Izx,Izz,g,m,p,phi_r,psi_r,q, ...
    r,rho,theta_r,u,u_T,u_phi,u_psi,u_theta,v,w);

%Integrate x_dot equation (given in Eqn ***)
dx = x_dot;
ulqr = [u_T;u_phi;u_theta;u_psi];
end