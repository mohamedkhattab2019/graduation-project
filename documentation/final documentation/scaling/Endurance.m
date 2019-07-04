clc ; clear vars ; close all ;

load('CL_CD');
% Rt=1;
Cdo = 0.0036; %we don't need to use it
k = 0.0545; %we don't need to use it (from polyfit) figure 1
rho =1.225;
S = .12 ;
u = 1:.05:50;
Rt=1;
V = 14.8;
W = 2.5*9.8;
% V = 11.1;
% W = 1.9*9.8;
CLn = (2*W/rho/S)./u.^2;
for i=1:length(u)
Cd(i) = interp1(WingPolarGraph3(:,1),WingPolarGraph3(:,2),CLn(i));
end

figure
hold all
plot (CLn(1:20:end),Cd(1:20:end),'*','LineWidth',2)
plot(CLn(210:end),0.0545*CLn(210:end).^2-0.0013*CLn(210:end),'--','LineWidth',2)
plot(WingPolarGraph3(:,1),WingPolarGraph3(:,2),'LineWidth',2)
grid on
legend('Linear interpolation','2nd order pol fit','Actual Data')
title('CL-CD')
xlabel('CL')
ylabel('CD')

figure
hold all
eta = 0.6;
% c=4 ,n=1
C1 = 4;
n1 = 1;
E1=Rt^(1-n1)*(eta*V*C1./(.5*rho*u.^3*S.*Cd)).^n1;

plot(u,E1,'r','LineWidth',1.5)

% c=4 ,n=1.3
C2 = 4;
n2 = 1.3;
E2=Rt^(1-n2)*(eta*V*C2./(.5*rho*u.^3*S.*Cd)).^n2;
plot(u,E2,'r--','LineWidth',1.5)


% c=5 ,n=1
C3 = 5;
n3 = 1;
E3=Rt^(1-n3)*(eta*V*C3./(.5*rho*u.^3*S.*Cd)).^n3;
plot(u,E3,'m','LineWidth',1.5)


% c=5 ,n=1.3
C4 = 5;
n4 = 1.3;
E4=Rt^(1-n4)*(eta*V*C4./(.5*rho*u.^3*S.*Cd)).^n4;
plot(u,E4,'m--','LineWidth',1.5)
% c=6 ,n=1
C5 = 6;
n5 = 1;
E5=Rt^(1-n5)*(eta*V*C5./(.5*rho*u.^3*S.*Cd)).^n5;
plot(u,E5,'b','LineWidth',1.5)

% c=6 ,n=1.3
C6 = 6;
n6 = 1.3;
E6=Rt^(1-n6)*(eta*V*C6./(.5*rho*u.^3*S.*Cd)).^n6;
plot(u,E6,'b--','LineWidth',1.5)
I1 =(C1./Rt).*(Rt./E1).^(1/n1);
plot(u,I1,'k--','LineWidth',2)


legend('E:C=4 ,n=1','E:C=4,n=1.3','E:C=5, n=1','E:C=5, n=1.3','E:C=6,n=1','E:C=6,n=1.3','I for any C,n')

plot(33.3*[1 1],[0 7]) %max Endurance
plot(33.35*[1 1],[0 7],'--') %max Range

xlabel('v (m/s)')
ylabel('Endurance(hr) / I(Amp)')
title('Endurance')

grid minor

%% Range

% c=4 ,n=1
R1 =E1.*u;

% c=4 ,n=1.3
R2 =E2.*u;
% c=5 ,n=1
R3 =E3.*u;
% c=5 ,n=1.3
R4 =E4.*u;
% c=6 ,n=1
R5 =E5.*u;
% c=6 ,n=1.3
R6 =E6.*u;

figure
hold all
plot (u,R1,'b','LineWidth',1.5)
plot (u,R2,'b--','LineWidth',1.5)
plot (u,R3,'r','LineWidth',1.5)
plot (u,R4,'r--','LineWidth',1.5)
plot (u,R5,'k','LineWidth',2.5)
plot (u,R6,'k--','LineWidth',1.5)

legend('R:c=4 ,n=1','R:c=4 ,n=1.3','R:c=5 ,n=1','R:c=5,n=1.3','R:c=6,n=1','R:c=6,n=1.30')
plot(33.3*[1 1],[0 200]) %max Endurance
plot(33.35*[1 1],[0 200],'--') %max Range

grid minor 
 
 xlabel('v (m/s)')
ylabel('Range (Km)')
title('Range')