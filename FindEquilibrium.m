function Eq= FindEquilibrium(x)




%%%%%%%%%%%DELETE$%%%%%%%%%%%%%%
phi_d=15*pi/180;
vd=x(1);
wd=x(2);
uT=x(3);

rho=1.225;
g=9.81;
m=1.5;
Dy=0.3265;
Dz=2*.3265;

vr=sqrt(vd^2 + wd^2);

Eq(1)=vd*sin(phi_d)+wd*cos(phi_d);
Eq(2)=m*g*sin(phi_d)/m -.5*rho*Dy*vr*vd/m;
Eq(3)=m*g*cos(phi_d)/m -.5*rho*Dz*vr*wd/m + uT/m;
