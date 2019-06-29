clc ; clear vars ; close all ;
% data 
Ix = 0.12;
Iy = 0.16;
Iz = 0.23;
m = 1.9;
T =pi/180;
s = tf('s'); 
%% Roll
G_phi_dot= s/(Ix*s^2);
PID =   0.29577*(s+0.4938)/s;
roll_inner_loop = feedback(G_phi_dot*PID , 1);
inner_action = feedback(PID , G_phi_dot) ;
figure
subplot(2,1,1)
step(roll_inner_loop)
grid minor
subplot(2,1,2)
step(inner_action)
grid minor
%outer
G_outer = roll_inner_loop/s;
Kp = 1.1681;
roll_outer_loop = feedback(Kp *G_outer , 1);
outer_action = feedback(Kp ,G_outer );
figure
step(roll_outer_loop)
grid minor
title('\phi_{step_{response}}')
%final roll control action
roll_action = feedback(Kp*inner_action , G_phi_dot/s);
figure 
step(roll_action)
grid minor
title('\phi_{action}')

% %nonLinear
% input = 1;
% temp = lvm_import('roll_response.lvm',0);
% nonL_roll_response = temp.Segment1.data;
% figure
% subplot(2,1,1)
% hold all
% plot(nonL_roll_response(:,1),nonL_roll_response(:,2),'LineWidth',1.5)
% step(input*roll_outer_loop,20)
% legend('NonLinear','Linear')
% title('Roll step response')
% ylabel('phi(deg)')
% grid minor
% 
% temp = lvm_import('roll_action.lvm',0);
% nonL_roll_action = temp.Segment1.data;
% subplot(2,1,2)
% hold all
% plot(nonL_roll_action(:,1),nonL_roll_action(:,2),'LineWidth',1.5)
% step(input*roll_action*T,20)
% legend('NonLinear','Linear')
% title('Roll step control action')
% ylabel('Roll Torque (N.m)')
% grid minor
%% theta/pitch
G_theta_dot = s/(Iy*s^2);
PID = 0.31599*(s+0.4938)/s;
pitch__inner_loop = feedback(G_theta_dot*PID , 1);
inner_action = feedback(PID , G_theta_dot) ;
figure
subplot(2,1,1)
step(pitch__inner_loop)
grid minor
subplot(2,1,2)
step(inner_action)
grid minor
% outer
G_out_th = pitch__inner_loop/s;
Kp= 0.89671;
pitch_outer_loop = feedback(Kp * G_out_th , 1);
figure
step(pitch_outer_loop)
grid minor
title('\theta_{step_{response}}')
%final Pitch control action
theta_action = feedback(Kp*inner_action , G_theta_dot/s);
figure 
step(theta_action*T)
grid minor
title('\theta_{action}')
%% yaw
G_psi_dot = s/(Iz*s^2);
PID = 0.71996*(s+0.7826)/s;
yaw_inner_loop = feedback(G_psi_dot*PID , 1);
inner_action = feedback(PID , G_psi_dot) ;
figure
subplot(2,1,1)
step(yaw_inner_loop)
grid minor
subplot(2,1,2)
step(inner_action)
grid minor
%outer 
G_out_psi = yaw_inner_loop/s;
Kp= 1.8681;
yaw_outer_loop = feedback(Kp * G_out_psi , 1);
figure
step(yaw_outer_loop)
grid minor
title('\psi_{step_{response}}')
%final yaw control action
Psi_action = feedback(Kp*inner_action , G_psi_dot/s);
figure 
step(Psi_action*T)
grid minor
title('\Psi_{action}')

%% Altitude Control
G_alt_dot = s/m/s^2;
PID = 3.753*(s+0.4938)/s;
alt_inner_loop = feedback(G_alt_dot*PID , 1);
inner_action = feedback(PID , G_alt_dot) ;
figure
subplot(2,1,1)
step(alt_inner_loop)
grid minor
subplot(2,1,2)
step(inner_action)
grid minor
%outer

G_al_outer = alt_inner_loop/s;
Kp =.86913;
alt_outer_loop = feedback(Kp *G_al_outer , 1);
outer_action = feedback(Kp ,G_al_outer );
figure
step(alt_outer_loop)
grid minor
title('h_{step_{response}}')
%final altitude control action
alt_action = feedback(Kp*inner_action , G_alt_dot/s);
figure 
step(alt_action)
grid minor
title('h_action')

%% x control
G_x_dot = -9.81*s/s^2 * pitch_outer_loop;
PID = -0.060006*(s+0.009831)/s;
X_inner_loop = feedback(G_x_dot*PID , 1);
inner_action = feedback(PID , G_x_dot) ;
figure
subplot(2,1,1)
step(X_inner_loop)
grid minor
subplot(2,1,2)
step(inner_action)
grid minor
%outer
G_X_outer = X_inner_loop/s;
Kp =.27023;
X_outer_loop = feedback(Kp *G_X_outer , 1);
outer_action = feedback(Kp ,G_X_outer );
figure
step(X_outer_loop)
grid minor
title('X_{step_{response}}')
%final altitude control action
X_action = feedback(Kp*inner_action , G_x_dot/s);
figure 
step(X_action)
grid minor
title('X_action')

%% y control

G_y_dot = -9.81*s/s^2 * roll_outer_loop;
PID = -0.074259*(s+0.01242)/s;
y_inner_loop = feedback(G_y_dot*PID , 1);
inner_action = feedback(PID , G_y_dot) ;
figure
subplot(2,1,1)
step(y_inner_loop)
grid minor
subplot(2,1,2)
step(inner_action)
grid minor

%outer
G_y_outer = y_inner_loop/s;
Kp = 0.37071;
y_outer_loop = feedback(Kp *G_y_outer ,1);
outer_action = feedback(Kp ,G_y_outer );
figure
step(y_outer_loop)
grid minor
title('Y_{step_{response}}')
%final altitude control action
y_action = feedback(Kp*inner_action , G_y_dot/s);
figure 
step(y_action)
grid minor
title('Y_action')



