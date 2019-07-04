clc; clear; close all;

rho =1.225;
S = 0.12 ;
Cl = 1.06;
Cd = 0.06;

V = 15.4; %volt

throttle_data = [0.25 0.375 0.5 0.625 0.75 0.875 1];
I_data = 4*[0.9 2.1 4.1 6.9 10.7 15.8 20.9];     %A
F_data = 4*9.81*[0.19 0.34 0.54 0.79 1.06 1.37 1.62]; %N
u_max = sqrt((3*9.81 - min(F_data))/0.5/rho/S/Cl)-0.2;
u_LO = sqrt((3*9.81)/0.5/rho/S/Cl)-0.2;

F_pusher = 0.5*9.81;

dt = 0.1;
tfinal = 0.5*60*60; %sec
%initialization
i = 2;
SOC(1) = 100;   SOCquadonly(1) = 100; SOCfixedonly(1) = 100; SOCfixedonly2(1) = 100;
Fquad(1) = max(F_data);
Iquad(1) = max(I_data);
Iwing(1) = 0;
I(1) = Iquad(1) + Iwing(1); Iquadonly(1) = Iquad(1); Ifixedonly(1) = 0; Ifixedonly2(1) = 0;
u(1) = 0; uquadonly(1) = 0; ufixedonly(1) = 0; ufixedonly2(1) = 0; 
Fwing(1) = 0;   Fquadonly(1)= Fquad(1); Ffixedonly(1)= 0; Ffixedonly2(1)= 0;
x(1) = 0;   xquadonly(1)=0; xfixedonly(1)=0; xfixedonly2(1)=0;
for t = dt:dt:tfinal
    
    %%%Quad-plane
    if t<=15
        %velocity
        u(i) = 0;
        %wing
        Fwing(i) = 0;
        Iwing(i) = 0;
        %quad
        Fquad(i) = max(F_data);
        Iquad(i) = max(I_data);
    else
        if (u(i-1) < u_max)
            %velocity
            u(i) = u(i-1) + ( F_pusher - 0.5*rho*u(i-1)^2*S*Cd )*dt;
            %wing
            Iwing(i) = F_pusher*u(i) / V;
            Fwing(i) = 0.5*rho*u(i)^2*S*Cl;
            %quad
            Fquad(i) = 3*9.81 - Fwing(i);
            if (Fquad(i) < min(F_data))
                Fquad(i) = min(F_data);
            end
            Iquad(i) = interp1(F_data,I_data,Fquad(i));
        else
            %velocity
            u(i) = u(i-1);
            %wing
            Iwing(i) = 0.5*rho*u(i)^3*S*Cd / V;
            Fwing(i) = 0.5*rho*u(i)^2*S*Cl;
            %quad
            Fquad(i) = 3*9.81 - Fwing(i);
            Iquad(i) = interp1(F_data,I_data,Fquad(i));
        end   
        
    end
    
    I(i) = Iquad(i) + Iwing(i);
    SOC(i) = Battery(I(i),SOC(i-1),dt);
    x(i) = x(i-1) + u(i)*dt;
    %------------------------------------------------------------------
    %%%Quad_only
     if t<=15
        %velocity
        uquadonly(i) = 0;
        %quad
        Fquadonly(i) = max(F_data);
        Iquadonly(i) = max(I_data);
     else
        %velocity
        if ((uquadonly(i-1) < u_max)&&(SOCquadonly(i-1) > 10))
            uquadonly(i) = uquadonly(i-1) + ( 4*sind(2)*9.81 - 0.5*rho*uquadonly(i-1)^2*S*Cd)*dt;
            Iquadonly(i) = interp1(F_data,I_data,3.2*9.81);
        elseif ( SOCquadonly(i-1) < 10)
            uquadonly(i) = uquadonly(i-1) - ( 0.5*rho*uquadonly(i-1)^2*S*Cd)*dt;
            Iquadonly(i) = 0;   
            if(uquadonly(i) < 0)
                uquadonly(i) = 0;
            end
        else
            uquadonly(i) = u_max;  
            Iquadonly(i) = interp1(F_data,I_data,3.1*9.81);
        end
        %Fquadonly(i) = 3.1*9.81;
        
    end
    
    SOCquadonly(i) = Battery(Iquadonly(i),SOCquadonly(i-1),dt);
    xquadonly(i) = xquadonly(i-1) + uquadonly(i)*dt;
    
    %-----------------------------------------------------------
    %%%Fixed wing
     if ufixedonly<=u_LO
        %velocity
        %Rolling Resistance coeff
        eta_rr=0.01;
        ufixedonly(i) = ufixedonly(i-1) + ( F_pusher - 0.5*rho*ufixedonly(i-1)^2*S*Cd - eta_rr*3*9.81)*dt;
        %Ffixedonly(i) = F_pusher*ufixedonly(i) / V;
        Ifixedonly(i) = F_pusher*ufixedonly(i) / V;
     else
        %velocity
        ufixedonly(i) = u_LO;  
        Ifixedonly(i) = 0.5*rho*ufixedonly(i)^3*S*Cd / V;
        Ffixedonly(i) = 0.5*rho*ufixedonly(i)^2*S*Cl;
    end
    
    SOCfixedonly(i) = Battery(Ifixedonly(i),SOCfixedonly(i-1),dt);
    xfixedonly(i) = xfixedonly(i-1) + ufixedonly(i)*dt;
    %----------------------------------------------------------
    %%%Fixed wing2
     if ufixedonly2<=u_max
        %velocity
        %Rolling Resistance coeff
        eta_rr=0.01;
        ufixedonly2(i) = ufixedonly2(i-1) + ( F_pusher - 0.5*rho*ufixedonly2(i-1)^2*S*Cd - eta_rr*3*9.81)*dt;
        %Ffixedonly(i) = F_pusher*ufixedonly(i) / V;
        Ifixedonly2(i) = F_pusher*ufixedonly2(i) / V;
     else
        %velocity
        ufixedonly2(i) = u_max;  
        Ifixedonly2(i) = 0.5*rho*ufixedonly2(i)^3*S*Cd / V;
        Ffixedonly2(i) = 0.5*rho*ufixedonly2(i)^2*S*Cl;
    end
    
    SOCfixedonly2(i) = Battery(Ifixedonly2(i),SOCfixedonly2(i-1),dt);
    xfixedonly2(i) = xfixedonly2(i-1) + ufixedonly2(i)*dt;
    
i = i + 1;
end
t = 0:dt:tfinal;

figure

subplot(2,2,1)
hold all
plot(t./60./60,u,'LineWidth',3)
plot(t./60./60,uquadonly,'--','LineWidth',2)
plot(t./60./60,ufixedonly,'--','LineWidth',2)
plot(t./60./60,ufixedonly2,'--','LineWidth',2)
legend('Quad-plane','Quad-copter','FixedWing-SameWeight','FixedWing-SameSpeed')
ylabel('Speed (m/s)')
xlabel('t (hr)')
grid on

subplot(2,2,2)
hold all
plot(t./60./60,SOC,'LineWidth',3)
plot(t./60./60,SOCquadonly,'--','LineWidth',2)
plot(t./60./60,SOCfixedonly,'--','LineWidth',2)
plot(t./60./60,SOCfixedonly2,'--','LineWidth',2)
ylabel('SOC(%)')
xlabel('t (hr)')
grid on

subplot(2,2,3)
hold all
plot(t./60./60,I,'LineWidth',3)
plot(t./60./60,Iquadonly,'--','LineWidth',2)
plot(t./60./60,Ifixedonly,'--','LineWidth',2)
plot(t./60./60,Ifixedonly2,'--','LineWidth',2)
ylabel('I (A)')
xlabel('t (hr)')
grid on

subplot(2,2,4)
hold all
plot(t./60./60,x/1000,'LineWidth',3)
plot(t./60./60,xquadonly/1000,'--','LineWidth',2)
plot(t./60./60,xfixedonly/1000,'--','LineWidth',2)
plot(t./60./60,xfixedonly2/1000,'--','LineWidth',2)
ylabel('Range (Km)')
xlabel('t (hr)')
grid on

