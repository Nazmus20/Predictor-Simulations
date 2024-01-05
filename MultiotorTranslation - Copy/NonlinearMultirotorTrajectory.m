function [x_dot, ucntrl] = NonlinearMultirotorTrajectory(tdum, xdum, fxu, ...
    Klqr, Values, Sim)

% NonlinearMultirotorTrajectory.m
%
% Nonlinear multirotor trajectory at some specified values.
% Initially starts from steady-state/hover. If an LQR controller is used 
% then the input is ue=uref - Klqr*(x - xref) which replaces the inputs 
% u in f(x,u) The state xe=x-x_ref is used in place of x in f(x,u)

%%%INPUTS%%%
% tdum: ode45() evaluation time steps
% xdum: ode45() evaluated solutions 
% A: Linearized state matrix
% B: Linearized input matrix
% Klqr: The LQR gain matrix obtained from the linearized EOMs, applied to
% control a nonlinear system
% Sim: Simulation parameters included in a structure
% Sim.Ts: Sampling time, s
% Sim.t_vec: Time vector, s
% Sim.out_del: Outgoing delay between the operator's command and the
% command reaching the system, s
% Sim.in_del: Incoming delay between the system's output and the output 
% reaching the operator, s
% Sim.out_delDT: Discrete Time (DT) converions of Sim.out_del
% Sim.in_delDT: Discrete Time (DT) converions of Sim.in_del
% Sim.initial_state: Initial states of the system, usually the steady state
% Sim.initial_input: Initial inputs of the system, usually the steady input

%%%OUTPUTS%%%
% dx: State derivatives of the 12-states at each t_vec step
% ucntrl: The control input of the controller to the UAV at each t_vec step

if (tdum<=Sim.in_del+1)
    des_state = Sim.initial_state;
    des_input = Sim.initial_input;
else 
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

u_Tref = interp1(Sim.t_vec, Sim.del_des_input(1,:), tdum);
u_phiref = interp1(Sim.t_vec, Sim.del_des_input(2,:), tdum);
u_thetaref = interp1(Sim.t_vec, Sim.del_des_input(3,:), tdum);
u_psiref = interp1(Sim.t_vec, Sim.del_des_input(4,:), tdum);
des_input = [u_Tref; u_phiref; u_thetaref; u_psiref];
end

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
end