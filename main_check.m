close all;
clearvars;
global I % moment of intertia matrix 
global beta % ALPHA  parameter for quaternion trajectory 
%ALPHA=6*1e-3; 
beta=1e-2;
t1=0; % initial time
t2=100; % final time for the typical integration
h=0.01; % time step
wd=[0;beta;0];
I=eye(3);% declaring the moment of inertia
qBI0=[0,0,0,1]';  % initial quaternion 
qOI0=q_orbit(t1);
qBO0=quat_prod_onenorm(qBI0,quat_inv(qOI0));
wBIB0=1e-3*[0,-1,1]'; % initial omega
wBOB0=wBIB0-quat2Rot(qBO0)*wd;
%% Typical Rk4 integrationin BI
 F=@Dynamics;
 [time, state]=rk4(F,h,t1,t2,[qBI0;wBIB0]);
%% Typical RK4 integration in BO
G=@Dynamics3;
[time2, state2]=rk4(G,h,t1,t2,[qBO0;wBOB0]);
N=length(time);
q_true=zeros(N,4);
q_BI=state(:,1:4);
q_BO=state2(:,1:4);
q_BI2BO=zeros(N,4);
w_BIB=state(:,5:7);
w_BOB=state2(:,5:7);
w_BIB2BOB=zeros(N,3);
for i=1:N
    q_BI2BO(i,:)=quat_prod_onenorm(q_BI(i,:)',quat_inv(q_orbit(time(i,1))))';
    w_BIB2BOB(i,:)=(w_BIB(i,:)'-quat2Rot(q_BO(i,:)')*wd)';
end
%% plotting the results
setGraphics;
%------------------------------------------
%q_BO=q_BO-q_BI2BO;
figure(3);
subplot(2,2,1);
plot(time,q_BO(:,1));
title('first comp.') ;
xlabel('time in sec');
ylabel('q_{BO}(1)');
hold on;
subplot(2,2,2);
plot(time,q_BO(:,2));
title('second comp.') ;
xlabel('time in sec');
ylabel('q_{BO}(2)');
hold on;
subplot(2,2,3);
plot(time,q_BO(:,3));
title('third comp.') ;
xlabel('time in sec');
ylabel('q_{BO}(3)');
hold on;
subplot(2,2,4);
plot(time,q_BO(:,4));
title('fourth comp.') ;
xlabel('time in sec');
ylabel('q_{BO}(4)');
hold on;
%print(gcf,'quat_err_rk4.pdf','-dpdf','-bestfit','-r400');
% figure(4);
subplot(2,2,1);
plot(time,q_BI2BO(:,1));
title('first comp.') ;
xlabel('time in sec');
ylabel('q_{BI2BO}(1)');
subplot(2,2,2);
plot(time,q_BI2BO(:,2));
title('second comp. ') ;
xlabel('time in sec');
ylabel('q_{BI2BO}(2)');
subplot(2,2,3);
plot(time,q_BI2BO(:,3));
title('third comp.') ;
xlabel('time in sec');
ylabel('q_{BI2BO}(3)');
xlabel('time in sec');
subplot(2,2,4);
plot(time,q_BI2BO(:,4));
title('fourth comp.') ;
xlabel('time in sec');
ylabel('q_{BI2BO}(4)');
%print(gcf,'quat_err_Geom.pdf','-dpdf','-bestfit','-r400');
%---------------------------------------------------------
% figure(5)
% plot(time,del_theta_rk4);
% hold on;
% plot(time,del_theta_Geom);
% xlabel('time in sec');
% ylabel('angle in radians');
% legend({'$\delta\theta_{rk4}$','$\delta\theta_{Geom}$'},'Interpreter','latex','Location','best');
% title('Roataion angle of the error quaterion');
% % print(gcf,'angleOfErrQuat.pdf','-dpdf','-bestfit','-r400');
% % 
% % save del_theta_rk4_2.mat del_theta_rk4;
% % save del_theta_Geom_2.mat del_theta_Geom;
