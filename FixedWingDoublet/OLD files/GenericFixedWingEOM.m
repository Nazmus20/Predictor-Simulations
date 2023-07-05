function xdot = GenericFixedWingEOM(t,x)

% GenericFixedWingEOM.m
%
% Nonlinear equations of motion for a rigid, fixed-wing airplane.


global e1 e2 e3 rho m g S b c Inertia Power ...
Cx0 Cxu Cxw Cxw2 ...
Cz0 Czw Czw2 Czq Czde ...
Cm0 Cmw Cmq Cmde ...
Cy0 Cyv Cyp Cyr Cyda Cydr ...
Cl0 Clv Clp Clr Clda Cldr ...
Cn0 Cnv Cnv2 Cnp Cnr Cnda Cndr ...
dTEq deEq ...
VonKarmanFlag Amp Phase Omega Phase2 Omega2 

% Parse out state vector components
X = x(1:3);
Theta = x(4:6);
    phi = Theta(1);
    theta = Theta(2);
    psi = Theta(3);
V = x(7:9);
    u = V(1);
    v = V(2);
    w = V(3);    
omega = x(10:12);
    p = omega(1);
    q = omega(2);
    r = omega(3);    

RIB = expm(psi*hat(e3))*expm(theta*hat(e2))*expm(phi*hat(e1));
LIB = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
    0, cos(phi), -sin(phi);
    0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

if VonKarmanFlag == 1

%%% WIND VELOCITY (1D VON KARMAN) %%%
WX = Amp(:,1)'*cos(Omega*X(1)+Phase(:,1));
WY = Amp(:,2)'*cos(Omega*X(1)+Phase(:,2));
WZ = Amp(:,3)'*cos(Omega*X(1)+Phase(:,3));
W = [WX; WY; WZ];

%%% WIND GRADIENT (1D VON KARMAN) %%%
WXX = -(Amp(:,1).*Omega)'*sin(Omega*X(1)+Phase(:,1));
WYX = -(Amp(:,2).*Omega)'*sin(Omega*X(1)+Phase(:,2));
WZX = -(Amp(:,3).*Omega)'*sin(Omega*X(1)+Phase(:,3));
GradW = [WXX, 0, 0; WYX, 0, 0; WZX, 0, 0];

elseif VonKarmanFlag == 2
    
%%% WIND VELOCITY (2D VON KARMAN) %%%
WX = Amp(:,1)'*(cos(Omega*X(1)+Phase(:,1)).*cos(Omega2*X(2)+Phase2(:,1)));
WY = Amp(:,2)'*(cos(Omega*X(1)+Phase(:,2)).*cos(Omega2*X(2)+Phase2(:,2)));
WZ = Amp(:,3)'*(cos(Omega*X(1)+Phase(:,3)).*cos(Omega2*X(2)+Phase2(:,3)));;
W = [WX; WY; WZ];

%%% WIND GRADIENT (2D VON KARMAN) %%%
WXX = -(Amp(:,1).*Omega)'*(sin(Omega*X(1)+Phase(:,1)).*cos(Omega2*X(2)+Phase2(:,1)));
WYX = -(Amp(:,2).*Omega)'*(sin(Omega*X(1)+Phase(:,1)).*cos(Omega2*X(2)+Phase2(:,1)));
WZX = -(Amp(:,3).*Omega)'*(sin(Omega*X(1)+Phase(:,1)).*cos(Omega2*X(2)+Phase2(:,1)));
WXY = -(Amp(:,1).*Omega2)'*(cos(Omega*X(1)+Phase(:,1)).*sin(Omega2*X(2)+Phase2(:,1)));
WYY = -(Amp(:,2).*Omega2)'*(cos(Omega*X(1)+Phase(:,1)).*sin(Omega2*X(2)+Phase2(:,1)));
WZY = -(Amp(:,3).*Omega2)'*(cos(Omega*X(1)+Phase(:,1)).*sin(Omega2*X(2)+Phase2(:,1)));
GradW = [WXX, WXY, 0; WYX, WYY, 0; WZX, WZY, 0];

else
    W = [0;0;0]; GradW = zeros(3);
    
end

% Wind velocity in body frame, air-relative velocity, and dynamic pressure
W = RIB'*W; % Convert wind velocity to body frame
Vr = V - W;
    ur = Vr(1);
    vr = Vr(2);
    wr = Vr(3); 
PDyn = (1/2)*rho*norm(Vr)^2;

% Wind gradient in body frame
Phi = RIB'*GradW'*RIB;

% Effective angular velocity of the fluid (Note: Etkin argues for 
% a different formulation of the "angular velocity of the wind")
temp = -(Phi-Phi');
omegaf = [-temp(2,3); temp(1,3); -temp(1,2)]; 
omegar = omega - omegaf;
    pr = omegar(1);
    qr = omegar(2);
    rr = omegar(3);

%%% AERODYNAMIC FORCES AND MOMENTS %%%

% Aerodynamic angles 
%beta = asin(Vr(2)/norm(Vr));
%alpha = atan2(Vr(3),Vr(1));

% Control deflections
de = deEq;
da = 0;
da = - 0.5*phi - 2*pr; % Add some roll stiffness and damping
dr = 0;
dT = dTEq;

% Aerodynamic force coefficients
Cx = Cx0 + Cxu*(ur/norm(Vr)) + Cxw*(wr/norm(Vr)) + Cxw2*(wr/norm(Vr))^2; 
Cy = Cy0 + Cyv*(vr/norm(Vr)) + Cyp*((b*pr)/(2*norm(V))) + Cyr*((b*rr)/(2*norm(Vr))) ...
    + Cyda*da + Cydr*dr;
Cz = Cz0 + Czw*(wr/norm(Vr)) + Czw2*(wr/norm(Vr))^2 + Czq*((c*qr)/(2*norm(Vr))) ...
    + Czde*de;

Thrust = dT*Power/norm(Vr);  % Constant power
X = PDyn*S*Cx + Thrust;
Y = PDyn*S*Cy;
Z = PDyn*S*Cz;
Force_Aero = [X; Y; Z];

% Components of aerodynamic moment (modulo "unsteady" terms)
Cl = Cl0 + Clv*(vr/norm(Vr)) + Clp*((b*pr)/(2*norm(Vr))) + Clr*((b*rr)/(2*norm(Vr))) + ...
    Clda*da + Cldr*dr;
Cm = Cm0 + Cmw*(wr/norm(Vr)) + Cmq*((c*qr)/(2*norm(Vr))) + Cmde*de;
Cn = Cn0 + Cnv*(vr/norm(Vr)) + Cnv2*(vr/norm(Vr))^2 + Cnp*((b*pr)/(2*norm(Vr))) + Cnr*((b*rr)/(2*norm(Vr))) + ...
    Cnda*da + Cndr*dr;

L = PDyn*S*b*Cl;
M = PDyn*S*c*Cm;
N = PDyn*S*b*Cn;
Moment_Aero = [L; M; N];

% Sum of forces and moments
Force = RIB'*(m*g*e3) + Force_Aero;
Moment = Moment_Aero;

%%% EQUATIONS OF MOTION %%%

%%% KINEMATIC EQUATIONS %%%
XDot = RIB*V;
ThetaDot = LIB*omega;

%%% DYNAMIC EQUATIONS %%%
VDot = (1/m)*(cross(m*V,omega) + Force);
omegaDot = inv(Inertia)*(cross(Inertia*omega,omega) + Moment);

xdot = [XDot; ThetaDot; VDot; omegaDot];
