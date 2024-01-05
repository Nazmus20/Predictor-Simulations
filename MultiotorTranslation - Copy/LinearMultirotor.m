function [A, B] = LinearMultirotor(varargin)

% LinearMultirotor.m
%
% Linearizes multirotor EOM at some specified steady values

%%%INPUTS%%%
% fxu: The 12 nonlinear symbolic EOMs, obtained from MultirotorEOM()
% Param: A structure containing symbolic parameters like mass, gravity,
% states etc. for symbolic computation
% Param.m: Mass, [kg]
% Param.g: Acceleration due to gravity, [m/s^2]
% Param.I: Moment of inertia matrix, [kg-m^2]
% Param.states: The 12-symbolic states of the system to take the derivative
% Param.inputs: The 4-symbolic inputs of the system to take the derivative
% Val: Another structure similar to "Param" but containing the numerical
% values
% Val.m: Mass, [kg]
% Val.g: Acceleration due to gravity, [m/s^2]
% Val.W: Weight, [N]
% Val.rho: Density, [kg/m^3]
% Val.I: Moment of inertia matrix, [kg-m^2]
% Val.steady_states: The 12-numeric steady-states of the system to evaluate
% the symbolic derivatives
% Param.steady_inputs: The 4-numeric steady-inputs of the system to
% evaluate the symbolic derivatives

%%%OUTPUTS%%%
% A: Linearized state matrix in symbolic/numeric variables depending on the
% number of parameters passed
% B: Linearized input matrix in symbolic/numeric variables depending on the
% number of parameters passed

%Defining the 12 states steady state values in symbolic, to be evaluated
syms x_ss y_ss z_ss phi_ss theta_ss psi_ss u_ss v_ss w_ss p_ss q_ss ...
    r_ss u_T_ss u_phi_ss u_theta_ss u_psi_ss real

if nargin==2
    fxu=varargin{1}; Param=varargin{2}; 
    
    %A = dfxu/dx @ x=xss, u=uss
    A = jacobian(fxu, [Param.states]);
    A = subs(A, [Param.states; Param.inputs], [x_ss; y_ss; z_ss; ...
        phi_ss; theta_ss; psi_ss; u_ss; v_ss; w_ss; p_ss; q_ss; r_ss; ...
        u_T_ss; u_phi_ss; u_theta_ss; u_psi_ss]);
    
    %B = dfxu/du @ x=val(1:12), u=val(13:16)
    B = jacobian(fxu, [Param.inputs]);
    B = subs(B, [Param.states; Param.inputs], [x_ss; y_ss; z_ss; ...
        phi_ss; theta_ss; psi_ss; u_ss; v_ss; w_ss; p_ss; q_ss; r_ss; ...
        u_T_ss; u_phi_ss; u_theta_ss; u_psi_ss]);

elseif nargin==3
    fxu=varargin{1}; Param=varargin{2}; Val=varargin{3};
    
    %A = dfxu/dx @ x=val(1:12), u=val(13:16)
    A = jacobian(fxu, [Param.states]);
    A = subs(A, [Param.states; Param.inputs; Param.m; Param.g; ...
        Param.I(1,1); Param.I(2,2); Param.I(3,3); Param.I(1,2); ...
        Param.I(2,3); Param.I(1,3); Param.aero(1); Param.aero(2); ...
        Param.aero(3); Param.rho], ...
        [Val.steady_states; Val.steady_inputs; Val.m; ...
        Val.g; Val.I(1,1); Val.I(2,2); Val.I(3,3); Val.I(1,2); ...
        Val.I(2,3); Val.I(1,3); Val.aero(1); Val.aero(2); Val.aero(3); Val.rho]);
    A = double(A);
    
    %B = dfxu/du @ x=val(1:12), u=val(13:16)
    B = jacobian(fxu, [Param.inputs]);
    B = subs(B,  [Param.states; Param.inputs; Param.m; Param.g; ...
        Param.I(1,1); Param.I(2,2); Param.I(3,3); Param.I(1,2); ...
        Param.I(2,3); Param.I(1,3); Param.aero(1); Param.aero(2); ...
        Param.aero(3); Param.rho], ...
        [Val.steady_states; Val.steady_inputs; Val.m; Val.g; ...
        Val.I(1,1); Val.I(2,2); Val.I(3,3); Val.I(1,2); Val.I(2,3); ...
        Val.I(1,3); Val.aero(1); Val.aero(2); Val.aero(3); Val.rho]);
    B = double(B);
else
    disp('LinearMultirotor.m accepts up to three input arguments')
end