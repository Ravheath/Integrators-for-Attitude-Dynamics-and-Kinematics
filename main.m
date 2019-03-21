close all;
clear all;
global I % moment of intertia matrix 
global ALPHA  % parameter for quaternion trajectory 
ALPHA=5.65*1e-3; 
t1=0; % initial time
t2=10; % final time for the typical integration
h=0.001; % time step 
I=eye(3);% declaring the moment of inertia
q0=quat_func_time(t1);  % initial quaternion 
w0=omega_func_time(t1); % initial omega 
%% Typical Rk4 integration
F=@Dynamics;
[time, state]=rk4(F,h,t1,t2,[q0;w0]);
%% New technique of integration
G=@Dynamics2;
h2=0.001; %  time step for the New integration
[time2, state2]=rk42(G,h2,t1,t2,[q0;w0]);
N=length(time);
q_true=zeros(N,4);
q_err_RK4=zeros(N,4);
q_err_Geom=zeros(N,4);
del_theta_RK4=zeros(N,1);
del_theta_Geom=zeros(N,1);
for i=1:N
    q_true(i,:)=quat_func_time(time(i,1))';
    q_err_RK4(i,:)=quat_prod(quat_inv(state(i,1:4)'),q_true(i,:)')';
    q_err_Geom(i,:)=quat_prod(quat_inv(state2(i,1:4)'),q_true(i,:)')';
    del_theta_RK4(i)=2*acos(abs(q_err_RK4(i,4)));
    del_theta_Geom(i)=2*acos(abs(q_err_Geom(i,4)));
%     q=[state(i,4),state(i,1:3)];
%     euler_true(i,:)=(180/pi)*quat2eul([q_true(i,4),q_true(i,1:3)],'ZYZ');
%     euler_state(i,:)=(180/pi)*quat2eul(q,'ZYZ');
    
end
%% plotting the results
set(groot,'defaultLineLineWidth',2)
set(0,'DefaultaxesLineWidth', 2)
set(0,'DefaultaxesFontSize', 16)
set(0,'DefaultTextFontSize', 12) 
set(0,'DefaultaxesFontName', 'arial') 
set(0,'defaultAxesXGrid','on')
set(0,'defaultAxesYGrid','on')
figure(1);
subplot(2,2,1);
plot(time,q_true(:,1)-state(:,1));
title('error in first component') ;
xlabel('time in sec');
ylabel('q_{true}(1)-q_{RK4}(1)');
subplot(2,2,2);
plot(time,q_true(:,2)-state(:,2));
title('error in second component') ;
xlabel('time in sec');
ylabel('q_{true}(2)-q_{RK4}(2)');
subplot(2,2,3);
plot(time,q_true(:,3)-state(:,3));
title('error in third component') ;
ylabel('q_{true}(3)-q_{RK4}(3)');
xlabel('time in sec');
subplot(2,2,4);
plot(time,q_true(:,4)-state(:,4));
title('error in fourth component') ;
xlabel('time in sec');
ylabel('q_{true}(4)-q_{RK4}(4)');
figure(2);
subplot(2,2,1);
plot(time2,q_true(:,1)-state2(:,1));
title('error in first component') ;
xlabel('time in sec');
ylabel('q_{true}(1)-q_{Geom}(1)');
subplot(2,2,2);
plot(time2,q_true(:,2)-state2(:,2));
title('error in second component') ;
xlabel('time in sec');
ylabel('q_{true}(2)-q_{Geom}(2)');
subplot(2,2,3);
plot(time2,q_true(:,3)-state2(:,3));
title('error in third component') ;
xlabel('time in sec');
ylabel('q_{true}(3)-q_{Geom}(3)');
subplot(2,2,4);
plot(time,q_true(:,4)-state2(:,4));
title('error in fourth component') ;
xlabel('time in sec');
ylabel('q_{true}(4)-q_{Geom}(4)');
%------------------------------------------
figure(3);
subplot(2,2,1);
plot(time,q_err_RK4(:,1));
title('first component') ;
xlabel('time in sec');
ylabel('q_{error,RK4}(1)');
subplot(2,2,2);
plot(time,q_err_RK4(:,2));
title('second component') ;
xlabel('time in sec');
ylabel('q_{error,RK4}(2)');
subplot(2,2,3);
plot(time,q_err_RK4(:,3));
title('third component') ;
xlabel('time in sec');
ylabel('q_{error,RK4}(3)');
subplot(2,2,4);
plot(time,q_err_RK4(:,4));
title('fourth component') ;
xlabel('time in sec');
ylabel('q_{error,RK4}(4)');
figure(4);
subplot(2,2,1);
plot(time,q_err_Geom(:,1));
title('first component ') ;
xlabel('time in sec');
ylabel('q_{error,Geom}(1)');
subplot(2,2,2);
plot(time,q_err_Geom(:,2));
title('second component ') ;
xlabel('time in sec');
ylabel('q_{error,Geom}(2)');
subplot(2,2,3);
plot(time,q_err_Geom(:,3));
title('third component ') ;
xlabel('time in sec');
ylabel('q_{error,Geom}(3)');
xlabel('time in sec');
subplot(2,2,4);
plot(time,q_err_Geom(:,4));
title('fourth component ') ;
xlabel('time in sec');
ylabel('q_{error,Geom}(4)');
%---------------------------------------------------------
figure(5)
plot(time,del_theta_RK4);
hold on;
plot(time,del_theta_Geom);
xlabel('time in sec');
ylabel('angle in radians');
legend({'$\delta\theta_{RK4}$','$\delta\theta_{Geom}$'},'Interpreter','latex','Location','best');
title('Roataion angle of the error quaterion in both the methods');

