close all;
clearvars;
global I % moment of intertia matrix 
global ALPHA  % parameter for quaternion trajectory 
ALPHA=6*1e-3; 
t1=0; % initial time
t2=3600; % final time for the typical integration
h=0.05; % time step 
I=eye(3);% declaring the moment of inertia
q0=quat_func_time(t1);  % initial quaternion 
R0=quat2Rot(q0);
w0=omega_func_time(t1); % initial omega 
%% New technique of integration
G=@Dynamics2;
[time, state]=rk4_geom(G,h,t1,t2,[R0,w0]);
N=length(time);
R_true=zeros(3*N,3);
R_err_Geom=zeros(3*N,3);
del_theta_Geom_SO3=zeros(N,1);
for i=1:N
    R_true(3*i-2:3*i,:)=quat2Rot(quat_func_time(time(i,1))');
    R_err_Geom(3*i-2:3*i,:)=R_true(3*i-2:3*i,:)*(state(3*i-2:3*i,1:3))';
    del_theta_Geom_SO3(i)=abs(acos(0.5*(trace(R_err_Geom(3*i-2:3*i,:))-1)));
end
%% plotting the results
set(groot,'defaultLineLineWidth',2)
set(0,'DefaultaxesLineWidth', 2)
set(0,'DefaultaxesFontSize', 16)
set(0,'DefaultTextFontSize', 12) 
set(0,'DefaultaxesFontName', 'arial') 
set(0,'defaultAxesXGrid','on')
% set(0,'defaultAxesYGrid','on')
% figure(1);
% subplot(2,2,1);
% plot(time,q_true(:,1)-state(:,1));
% title('error in first component') ;
% xlabel('time in sec');
% ylabel('q_{true}(1)-q_{RK4}(1)');
% subplot(2,2,2);
% plot(time,q_true(:,2)-state(:,2));
% title('error in second component') ;
% xlabel('time in sec');
% ylabel('q_{true}(2)-q_{RK4}(2)');
% subplot(2,2,3);
% plot(time,q_true(:,3)-state(:,3));
% title('error in third component') ;
% ylabel('q_{true}(3)-q_{RK4}(3)');
% xlabel('time in sec');
% subplot(2,2,4);
% plot(time,q_true(:,4)-state(:,4));
% title('error in fourth component') ;
% xlabel('time in sec');
% ylabel('q_{true}(4)-q_{RK4}(4)');
% figure(2);
% subplot(2,2,1);
% plot(time2,q_true(:,1)-state2(:,1));
% title('error in first component') ;
% xlabel('time in sec');
% ylabel('q_{true}(1)-q_{Geom}(1)');
% subplot(2,2,2);
% plot(time2,q_true(:,2)-state2(:,2));
% title('error in second component') ;
% xlabel('time in sec');
% ylabel('q_{true}(2)-q_{Geom}(2)');
% subplot(2,2,3);
% plot(time2,q_true(:,3)-state2(:,3));
% title('error in third component') ;
% xlabel('time in sec');
% ylabel('q_{true}(3)-q_{Geom}(3)');
% subplot(2,2,4);
% plot(time,q_true(:,4)-state2(:,4));
% title('error in fourth component') ;
% xlabel('time in sec');
% ylabel('q_{true}(4)-q_{Geom}(4)');
% %------------------------------------------
% figure(3);
% subplot(2,2,1);
% plot(time,q_err_RK4(:,1));
% title('first component') ;
% xlabel('time in sec');
% ylabel('q_{error,RK4}(1)');
% subplot(2,2,2);
% plot(time,q_err_RK4(:,2));
% title('second component') ;
% xlabel('time in sec');
% ylabel('q_{error,RK4}(2)');
% subplot(2,2,3);
% plot(time,q_err_RK4(:,3));
% title('third component') ;
% xlabel('time in sec');
% ylabel('q_{error,RK4}(3)');
% subplot(2,2,4);
% plot(time,q_err_RK4(:,4));
% title('fourth component') ;
% xlabel('time in sec');
% ylabel('q_{error,RK4}(4)');
% figure(4);
% subplot(2,2,1);
% plot(time,q_err_Geom(:,1));
% title('first component ') ;
% xlabel('time in sec');
% ylabel('q_{error,Geom}(1)');
% subplot(2,2,2);
% plot(time,q_err_Geom(:,2));
% title('second component ') ;
% xlabel('time in sec');
% ylabel('q_{error,Geom}(2)');
% subplot(2,2,3);
% plot(time,q_err_Geom(:,3));
% title('third component ') ;
% xlabel('time in sec');
% ylabel('q_{error,Geom}(3)');
% xlabel('time in sec');
% subplot(2,2,4);
% plot(time,q_err_Geom(:,4));
% title('fourth component ') ;
% xlabel('time in sec');
% ylabel('q_{error,Geom}(4)');
%---------------------------------------------------------
load del_theta_Geom_005.mat;
load del_theta_rk4_005.mat;

figure(5)
plot(time,del_theta_rk4);
hold on;
plot(time,del_theta_Geom);
plot(time,del_theta_Geom_SO3);
xlabel('time in sec');
ylabel('angle in radians');
legend({'$\delta\theta_{RK4}$','$\delta\theta_{Geom,quat}$','$\delta\theta_{Geom,SO(3)}$'},'Interpreter','latex','Location','best');
title('Roatation angle of the error attitude');
%print(gcf,'angleOfErrQuat.pdf','-dpdf','-bestfit','-r400');
