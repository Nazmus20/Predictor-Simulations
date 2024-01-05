function [x,Fk]=EKF_prediction(x,u,f,F,dt,nRK)
% This function is performs 4th order Runge-Kutta numerical integration
% given the following system
%
% xdot= f(x,u) + l(x,w,u)
% where l(x,w,u) is where noise enters the system
%
% and inputs
%
% Inputs
% x = state
% f = f(x), function of state/dynamics
% F = df(x)/dx, Jacobian of f w.r.t. x
% dt = time step
% nRK = number of Runge-Kutta integration steps

Fk=feval(F,x, u);
delt=dt/nRK;

    for kk=1:nRK
 %step 1
        x_k1=feval(f,x,u);
        Fev(:,:,1)=feval(F,x,u);
        F_k1=feval(F,x,u);

        %step 2
        x_k2=feval(f,x+x_k1*delt/2,u);
        Fev(:,:,2)=feval(F, x+x_k1*delt/2, u);
        F_k2=Fev(:,:,2);

        %step 3
        x_k3=feval(f,x+x_k2*delt/2,u);
        Fev(:,:,3)=feval(F, x+x_k2*delt/2, u);
        F_k3=Fev(:,:,3);

        %step 4
        x_k4=feval(f,x+delt*x_k3,u);
        Fev(:,:,4)=feval(F, x+delt*x_k3, u);  
        F_k4=Fev(:,:,4);

        x = x + delt*(x_k1 + 2*x_k2 + 2*x_k3 +x_k4)/6 ;
        Fk = Fk + delt*(F_k1 + 2*F_k2 + 2*F_k3 + F_k4)/6; 

    end
    x=x;
    Fk=Fk;
end

        %
