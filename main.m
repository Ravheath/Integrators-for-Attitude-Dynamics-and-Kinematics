close all;
clearvars;
global I % moment of intertia matrix 
global beta in1 wd kp kv in2 % ALPHA  parameter for quaternion trajectory 
%ALPHA=6*1e-3; 
beta=2*pi/6000;
% prompt1 = 'For uncontrolled press 1 and press 0 otherwise ';
% in1 = input(prompt1);
% prompt2 = 'For external torque press 1 and press 0 otherwise ';
% prompt5='For scaled Identity moment of inertia press 1 and for the actual moment of inertia press 0 ';
% in5=input(prompt5);
% in2=input(prompt2);
% prompt3='For how long do you want to run the simulation ';
% in3=input(prompt3);
% prompt4='Enter the time step ';
% in4=input(prompt4);
in5=0;
% actual moment of inertia 
for k=1:3
 if k==1
    in1=1;
    in2=0;
    in3=60000;
    in4=0.1;
    t1=0; % initial time
    t2=in3; % final time for the typical integration
    h=in4; % time step
    wd=[0;beta;0]; % omega OIO
    kp=0.0001;  %controller gains 
    kv=0.0001;%controller gains
    disp('The angular velocity w_OIO is ') 
    wd
    if in5==1
        I=0.001*eye(3);% declaring the moment of inertia
    else     
        % actual moment of inertia
        Ixx = 0.00152529;
        Iyy = 0.00145111;
        Izz = 0.001476;
        Ixy = 0.00000437;
        Iyz = - 0.00000408;
        Ixz = 0.00000118;
        I= [Ixx, Ixy, Ixz; Ixy, Iyy, Iyz; Ixz, Iyz, Izz];	
    end 
    qBI0=(1/(2))*[1,1,1,1]'  % initial quaternion 
    qOI0=q_orbit(t1)
    qBO0=quat_prod_onenorm(qBI0,quat_inv(qOI0))
    wBIB0=1e-3*[0,-1,1]' % initial omega
    wBOB0=wBIB0-quat2Rot(qBO0)*wd
    %%Integration 

    if in1==1 && in2==1 % uncontrolled and external torque
        F=@DynamicsBI;
        G=@DynamicsBO;
    elseif in1==1 && in2==0  %uncontrolled and no external torque
        F=@Dynamics_uncontrolledBI; 
        G=@Dynamics_uncontrolledBO;
    elseif  in1==0 && in2==0 % controlled and no external torque
        F=@DynamicsBI;
        G=@DynamicsBO;
    else
        disp('Not a valid choice of inputs')
        pause(5)
        exit
    end 
    % Typical Rk4 integrationin BI
    [time, state]=rk4(F,h,t1,t2,[qBI0;wBIB0]);
    %Typical RK4 integration in BO
    [time2, state2]=rk4(G,h,t1,t2,[qBO0;wBOB0]);
    N=length(time);
    q_true=zeros(N,4);
    q_BI=state(:,1:4);
    q_BO=state2(:,1:4);
    q_BI2BO=zeros(N,4);
    w_BIB=state(:,5:7);
    w_BOB=state2(:,5:7);
    w_BIB2BOB=zeros(N,3);
    q_err=zeros(N,4);
    del_theta=zeros(N,1);
    for i=1:N
        q_BI2BO(i,:)=quat_prod_onenorm(q_BI(i,:)',quat_inv(q_orbit(time(i,1))))';
        w_BIB2BOB(i,:)=(w_BIB(i,:)'-quat2Rot(q_BO(i,:)')*wd)';
        q_err(i,:)=quat_prod_onenorm(quat_inv(q_BI2BO(i,:)'),q_BO(i,:)')';
        del_theta(i)=2*acos(abs(q_err(i,4)));
    end
    %% plotting the results
    setGraphics;
    %------------------------------------------
    %q_BO=q_BO-q_BI2BO;
    figure(1);
    subplot(2,2,1);
    plot(time,q_BO(:,1));
    title('first comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BO}(1)');
    hold on;
    subplot(2,2,2);
    plot(time,q_BO(:,2));
    title('second comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BO}(2)');
    hold on;
    subplot(2,2,3);
    plot(time,q_BO(:,3));
    title('third comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BO}(3)');
    hold on;
    subplot(2,2,4);
    plot(time,q_BO(:,4));
    title('fourth comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BO}(4)');
    hold on;
    %print(gcf,'quat_err_rk4.pdf','-dpdf','-bestfit','-r400');
    % figure(4);
    subplot(2,2,1);
    plot(time,q_BI2BO(:,1));
    title('first comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BI2BO}(1)');
    subplot(2,2,2);
    plot(time,q_BI2BO(:,2));
    title('second comp. ') ;
    xlabel('time in sec');
    %ylabel('q_{BI2BO}(2)');
    subplot(2,2,3);
    plot(time,q_BI2BO(:,3));
    title('third comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BI2BO}(3)');
    xlabel('time in sec');
    subplot(2,2,4);
    plot(time,q_BI2BO(:,4));
    title('fourth comp.') ;
    xlabel('time in sec');
    % ylabel('q_{BI2BO}(4)');
    legend({'$q_{BO}$','$q_{BI2BO}$'},'Interpreter','latex','Location','best');
    %---------------------------------------------------------
    figure(2)
    plot(time,del_theta);
    xlabel('time in sec');
    ylabel('angle in radians');
    legend({'$\delta\theta_{q_{BO}, q_{BI2BO}}$'},'Interpreter','latex','Location','best');
    title({'Rotation angle of the error quaterion between $q_{BO}$  and $q_{BI2BO}$'},'Interpreter','latex');
    save results_wrong_uncont_nodist_actual.mat q_BO q_BI q_BI2BO 
   elseif k==2
    in1=1;
    in2=1;
    in3=60000;
    in4=0.01;
    t1=0; % initial time
    t2=in3; % final time for the typical integration
    h=in4; % time step
    wd=[0;beta;0];
    kp=0.0001;
    kv=0.0001;
    disp('The angular velocity w_OIO is ') 
    wd
    if in5==1
        I=0.001*eye(3);% declaring the moment of inertia
    else     
        Ixx = 0.00152529;
        Iyy = 0.00145111;
        Izz = 0.001476;
        Ixy = 0.00000437;
        Iyz = - 0.00000408;
        Ixz = 0.00000118;
        I= [Ixx, Ixy, Ixz; Ixy, Iyy, Iyz; Ixz, Iyz, Izz];	
    end 
    qBI0=(1/(2))*[1,1,1,1]'  % initial quaternion 
    qOI0=q_orbit(t1)
    qBO0=quat_prod_onenorm(qBI0,quat_inv(qOI0))
    wBIB0=1e-3*[0,-1,1]' % initial omega
    wBOB0=wBIB0-quat2Rot(qBO0)*wd
    %%Integration 

    if in1==1 && in2==1 % uncontrolled and external torque
        F=@DynamicsBI;
        G=@DynamicsBO;
    elseif in1==1 && in2==0  %uncontrolled and no external torque
        F=@Dynamics_uncontrolledBI; 
        G=@Dynamics_uncontrolledBO;
    elseif  in1==0 && in2==0 % controlled and no external torque
        F=@DynamicsBI;
        G=@DynamicsBO;
    else
        disp('Not a valid choice of inputs')
        pause(5)
        exit
    end 
    % Typical Rk4 integrationin BI
    [time, state]=rk4(F,h,t1,t2,[qBI0;wBIB0]);
    %Typical RK4 integration in BO
    [time2, state2]=rk4(G,h,t1,t2,[qBO0;wBOB0]);
    N=length(time);
    q_true=zeros(N,4);
    q_BI=state(:,1:4);
    q_BO=state2(:,1:4);
    q_BI2BO=zeros(N,4);
    w_BIB=state(:,5:7);
    w_BOB=state2(:,5:7);
    w_BIB2BOB=zeros(N,3);
    q_err=zeros(N,4);
    del_theta=zeros(N,1);
    for i=1:N
        q_BI2BO(i,:)=quat_prod_onenorm(q_BI(i,:)',quat_inv(q_orbit(time(i,1))))';
        w_BIB2BOB(i,:)=(w_BIB(i,:)'-quat2Rot(q_BO(i,:)')*wd)';
        q_err(i,:)=quat_prod_onenorm(quat_inv(q_BI2BO(i,:)'),q_BO(i,:)')';
        del_theta(i)=2*acos(abs(q_err(i,4)));
    end
    %% plotting the results
    setGraphics;
    %------------------------------------------
    %q_BO=q_BO-q_BI2BO;
    figure(3);
        subplot(2,2,1);
    plot(time(end-5000:end),q_BO(end-5000:end,1));
    title('first comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BO}(1)');
    hold on;
    subplot(2,2,2);
    plot(time(end-5000:end),q_BO(end-5000:end,2));
    title('second comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BO}(2)');
    hold on;
    subplot(2,2,3);
    plot(time(end-5000:end),q_BO(end-5000:end,3));
    title('third comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BO}(3)');
    hold on;
    subplot(2,2,4);
    plot(time(end-5000:end),q_BO(end-5000:end,4));
    title('fourth comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BO}(4)');
    hold on;
    %print(gcf,'quat_err_rk4.pdf','-dpdf','-bestfit','-r400');
    % figure(4);
    subplot(2,2,1);
    plot(time(end-5000:end),q_BI2BO(end-5000:end,1));
    title('first comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BI2BO}(1)');
    subplot(2,2,2);
    plot(time(end-5000:end),q_BI2BO(end-5000:end,2));
    title('second comp. ') ;
    xlabel('time in sec');
    %ylabel('q_{BI2BO}(2)');
    subplot(2,2,3);
    plot(time(end-5000:end),q_BI2BO(end-5000:end,3));
    title('third comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BI2BO}(3)');
    xlabel('time in sec');
    subplot(2,2,4);
    plot(time(end-5000:end),q_BI2BO(end-5000:end,4));
    title('fourth comp.') ;
    xlabel('time in sec');
    % ylabel('q_{BI2BO}(4)');
    legend({'$q_{BO}$','$q_{BI2BO}$'},'Interpreter','latex','Location','best');
    %---------------------------------------------------------
    figure(4)
    plot(time,del_theta);
    xlabel('time in sec');
    ylabel('angle in radians');
    legend({'$\delta\theta_{q_{BO}, q_{BI2BO}}$'},'Interpreter','latex','Location','best');
    title({'Rotation angle of the error quaterion between $q_{BO}$  and $q_{BI2BO}$'},'Interpreter','latex');
    save results_wrong_uncont_dist_actual.mat q_BO q_BI q_BI2BO 
 elseif k==3
    in1=0;
    in2=0;
    in3=200;
    in4=0.1;
    t1=0; % initial time
    t2=in3; % final time for the typical integration
    h=in4; % time step
    wd=[0;beta;0];
    kp=0.0001;
    kv=0.0001;
    disp('The angular velocity w_OIO is ') 
    wd
    if in5==1
        I=0.001*eye(3);% declaring the moment of inertia
    else     
        Ixx = 0.00152529;
        Iyy = 0.00145111;
        Izz = 0.001476;
        Ixy = 0.00000437;
        Iyz = - 0.00000408;
        Ixz = 0.00000118;
        I= [Ixx, Ixy, Ixz; Ixy, Iyy, Iyz; Ixz, Iyz, Izz];	
    end 
    qBI0=(1/(2))*[1,1,1,1]'  % initial quaternion 
    qOI0=q_orbit(t1)
    qBO0=quat_prod_onenorm(qBI0,quat_inv(qOI0))
    wBIB0=1e-3*[0,-1,1]' % initial omega
    wBOB0=wBIB0-quat2Rot(qBO0)*wd
    %%Integration 

    if in1==1 && in2==1 % uncontrolled and external torque
        F=@DynamicsBI;
        G=@DynamicsBO;
    elseif in1==1 && in2==0  %uncontrolled and no external torque
        F=@Dynamics_uncontrolledBI; 
        G=@Dynamics_uncontrolledBO;
    elseif  in1==0 && in2==0 % controlled and no external torque
        F=@DynamicsBI;
        G=@DynamicsBO;
    else
        disp('Not a valid choice of inputs')
        pause(5)
        exit
    end 
    % Typical Rk4 integrationin BI
    [time, state]=rk4(F,h,t1,t2,[qBI0;wBIB0]);
    %Typical RK4 integration in BO
    [time2, state2]=rk4(G,h,t1,t2,[qBO0;wBOB0]);
    N=length(time);
    q_true=zeros(N,4);
    q_BI=state(:,1:4);
    q_BO=state2(:,1:4);
    q_BI2BO=zeros(N,4);
    w_BIB=state(:,5:7);
    w_BOB=state2(:,5:7);
    w_BIB2BOB=zeros(N,3);
    q_err=zeros(N,4);
    del_theta=zeros(N,1);
    for i=1:N
        q_BI2BO(i,:)=quat_prod_onenorm(q_BI(i,:)',quat_inv(q_orbit(time(i,1))))';
        w_BIB2BOB(i,:)=(w_BIB(i,:)'-quat2Rot(q_BO(i,:)')*wd)';
        q_err(i,:)=quat_prod_onenorm(quat_inv(q_BI2BO(i,:)'),q_BO(i,:)')';
        del_theta(i)=2*acos(abs(q_err(i,4)));
    end
    %% plotting the results
    setGraphics;
    %------------------------------------------
    %q_BO=q_BO-q_BI2BO;
    figure(5);
    subplot(2,2,1);
    plot(time,q_BO(:,1));
    title('first comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BO}(1)');
    hold on;
    subplot(2,2,2);
    plot(time,q_BO(:,2));
    title('second comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BO}(2)');
    hold on;
    subplot(2,2,3);
    plot(time,q_BO(:,3));
    title('third comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BO}(3)');
    hold on;
    subplot(2,2,4);
    plot(time,q_BO(:,4));
    title('fourth comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BO}(4)');
    hold on;
    %print(gcf,'quat_err_rk4.pdf','-dpdf','-bestfit','-r400');
    % figure(4);
    subplot(2,2,1);
    plot(time,q_BI2BO(:,1));
    title('first comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BI2BO}(1)');
    subplot(2,2,2);
    plot(time,q_BI2BO(:,2));
    title('second comp. ') ;
    xlabel('time in sec');
    %ylabel('q_{BI2BO}(2)');
    subplot(2,2,3);
    plot(time,q_BI2BO(:,3));
    title('third comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BI2BO}(3)');
    xlabel('time in sec');
    subplot(2,2,4);
    plot(time,q_BI2BO(:,4));
    title('fourth comp.') ;
    xlabel('time in sec');
    % ylabel('q_{BI2BO}(4)');
    legend({'$q_{BO}$','$q_{BI2BO}$'},'Interpreter','latex','Location','best');
    %---------------------------------------------------------
    figure(6)
    plot(time,del_theta);
    xlabel('time in sec');
    ylabel('angle in radians');
    legend({'$\delta\theta_{q_{BO}, q_{BI2BO}}$'},'Interpreter','latex','Location','best');
    title({'Rotation angle of the error quaterion between $q_{BO}$  and $q_{BI2BO}$'},'Interpreter','latex');
    save results_wrong_cont_nodist_actual.mat q_BO q_BI q_BI2BO 
 end 
end 
%%

% Identity moment of inertia
in5=1;
for k=1:3
 if k==1
    in1=1;
    in2=0;
    in3=60000;
    in4=0.1;
    t1=0; % initial time
    t2=in3; % final time for the typical integration
    h=in4; % time step
    wd=[0;beta;0];
    kp=0.0001;
    kv=0.0001;
    disp('The angular velocity w_OIO is ') 
    wd
    if in5==1
        I=0.001*eye(3);% declaring the moment of inertia
    else     
        Ixx = 0.00152529;
        Iyy = 0.00145111;
        Izz = 0.001476;
        Ixy = 0.00000437;
        Iyz = - 0.00000408;
        Ixz = 0.00000118;
        I= [Ixx, Ixy, Ixz; Ixy, Iyy, Iyz; Ixz, Iyz, Izz];	
    end 
    qBI0=(1/(2))*[1,1,1,1]'  % initial quaternion 
    qOI0=q_orbit(t1)
    qBO0=quat_prod_onenorm(qBI0,quat_inv(qOI0))
    wBIB0=1e-3*[0,-1,1]' % initial omega
    wBOB0=wBIB0-quat2Rot(qBO0)*wd
    %%Integration 

    if in1==1 && in2==1 % uncontrolled and external torque
        F=@DynamicsBI;
        G=@DynamicsBO;
    elseif in1==1 && in2==0  %uncontrolled and no external torque
        F=@Dynamics_uncontrolledBI; 
        G=@Dynamics_uncontrolledBO;
    elseif  in1==0 && in2==0 % controlled and no external torque
        F=@DynamicsBI;
        G=@DynamicsBO;
    else
        disp('Not a valid choice of inputs')
        pause(5)
        exit
    end 
    % Typical Rk4 integrationin BI
    [time, state]=rk4(F,h,t1,t2,[qBI0;wBIB0]);
    %Typical RK4 integration in BO
    [time2, state2]=rk4(G,h,t1,t2,[qBO0;wBOB0]);
    N=length(time);
    q_true=zeros(N,4);
    q_BI=state(:,1:4);
    q_BO=state2(:,1:4);
    q_BI2BO=zeros(N,4);
    w_BIB=state(:,5:7);
    w_BOB=state2(:,5:7);
    w_BIB2BOB=zeros(N,3);
    q_err=zeros(N,4);
    del_theta=zeros(N,1);
    for i=1:N
        q_BI2BO(i,:)=quat_prod_onenorm(q_BI(i,:)',quat_inv(q_orbit(time(i,1))))';
        w_BIB2BOB(i,:)=(w_BIB(i,:)'-quat2Rot(q_BO(i,:)')*wd)';
        q_err(i,:)=quat_prod_onenorm(quat_inv(q_BI2BO(i,:)'),q_BO(i,:)')';
        del_theta(i)=2*acos(abs(q_err(i,4)));
    end
    %% plotting the results
    setGraphics;
    %------------------------------------------
    %q_BO=q_BO-q_BI2BO;
    figure(7);
    subplot(2,2,1);
    plot(time,q_BO(:,1));
    title('first comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BO}(1)');
    hold on;
    subplot(2,2,2);
    plot(time,q_BO(:,2));
    title('second comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BO}(2)');
    hold on;
    subplot(2,2,3);
    plot(time,q_BO(:,3));
    title('third comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BO}(3)');
    hold on;
    subplot(2,2,4);
    plot(time,q_BO(:,4));
    title('fourth comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BO}(4)');
    hold on;
    %print(gcf,'quat_err_rk4.pdf','-dpdf','-bestfit','-r400');
    % figure(4);
    subplot(2,2,1);
    plot(time,q_BI2BO(:,1));
    title('first comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BI2BO}(1)');
    subplot(2,2,2);
    plot(time,q_BI2BO(:,2));
    title('second comp. ') ;
    xlabel('time in sec');
    %ylabel('q_{BI2BO}(2)');
    subplot(2,2,3);
    plot(time,q_BI2BO(:,3));
    title('third comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BI2BO}(3)');
    xlabel('time in sec');
    subplot(2,2,4);
    plot(time,q_BI2BO(:,4));
    title('fourth comp.') ;
    xlabel('time in sec');
    % ylabel('q_{BI2BO}(4)');
    legend({'$q_{BO}$','$q_{BI2BO}$'},'Interpreter','latex','Location','best');
    %---------------------------------------------------------
    figure(8)
    plot(time,del_theta);
    xlabel('time in sec');
    ylabel('angle in radians');
    legend({'$\delta\theta_{q_{BO}, q_{BI2BO}}$'},'Interpreter','latex','Location','best');
    title({'Rotation angle of the error quaterion between $q_{BO}$  and $q_{BI2BO}$'},'Interpreter','latex');
    save results_wrong_uncont_nodist_ident.mat q_BO q_BI q_BI2BO 
   elseif k==2
    in1=1;
    in2=1;
    in3=60000;
    in4=0.01;
    t1=0; % initial time
    t2=in3; % final time for the typical integration
    h=in4; % time step
    wd=[0;beta;0];
    kp=0.0001;
    kv=0.0001;
    disp('The angular velocity w_OIO is ') 
    wd
    if in5==1
        I=0.001*eye(3);% declaring the moment of inertia
    else     
        Ixx = 0.00152529;
        Iyy = 0.00145111;
        Izz = 0.001476;
        Ixy = 0.00000437;
        Iyz = - 0.00000408;
        Ixz = 0.00000118;
        I= [Ixx, Ixy, Ixz; Ixy, Iyy, Iyz; Ixz, Iyz, Izz];	
    end 
    qBI0=(1/(2))*[1,1,1,1]'  % initial quaternion 
    qOI0=q_orbit(t1)
    qBO0=quat_prod_onenorm(qBI0,quat_inv(qOI0))
    wBIB0=1e-3*[0,-1,1]' % initial omega
    wBOB0=wBIB0-quat2Rot(qBO0)*wd
    %%Integration 

    if in1==1 && in2==1 % uncontrolled and external torque
        F=@DynamicsBI;
        G=@DynamicsBO;
    elseif in1==1 && in2==0  %uncontrolled and no external torque
        F=@Dynamics_uncontrolledBI; 
        G=@Dynamics_uncontrolledBO;
    elseif  in1==0 && in2==0 % controlled and no external torque
        F=@DynamicsBI;
        G=@DynamicsBO;
    else
        disp('Not a valid choice of inputs')
        pause(5)
        exit
    end 
    % Typical Rk4 integrationin BI
    [time, state]=rk4(F,h,t1,t2,[qBI0;wBIB0]);
    %Typical RK4 integration in BO
    [time2, state2]=rk4(G,h,t1,t2,[qBO0;wBOB0]);
    N=length(time);
    q_true=zeros(N,4);
    q_BI=state(:,1:4);
    q_BO=state2(:,1:4);
    q_BI2BO=zeros(N,4);
    w_BIB=state(:,5:7);
    w_BOB=state2(:,5:7);
    w_BIB2BOB=zeros(N,3);
    q_err=zeros(N,4);
    del_theta=zeros(N,1);
    for i=1:N
        q_BI2BO(i,:)=quat_prod_onenorm(q_BI(i,:)',quat_inv(q_orbit(time(i,1))))';
        w_BIB2BOB(i,:)=(w_BIB(i,:)'-quat2Rot(q_BO(i,:)')*wd)';
        q_err(i,:)=quat_prod_onenorm(quat_inv(q_BI2BO(i,:)'),q_BO(i,:)')';
        del_theta(i)=2*acos(abs(q_err(i,4)));
    end
    %% plotting the results
    setGraphics;
    %------------------------------------------
    %q_BO=q_BO-q_BI2BO;
    figure(9);
        subplot(2,2,1);
    plot(time(end-5000:end),q_BO(end-5000:end,1));
    title('first comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BO}(1)');
    hold on;
    subplot(2,2,2);
    plot(time(end-5000:end),q_BO(end-5000:end,2));
    title('second comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BO}(2)');
    hold on;
    subplot(2,2,3);
    plot(time(end-5000:end),q_BO(end-5000:end,3));
    title('third comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BO}(3)');
    hold on;
    subplot(2,2,4);
    plot(time(end-5000:end),q_BO(end-5000:end,4));
    title('fourth comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BO}(4)');
    hold on;
    %print(gcf,'quat_err_rk4.pdf','-dpdf','-bestfit','-r400');
    % figure(4);
    subplot(2,2,1);
    plot(time(end-5000:end),q_BI2BO(end-5000:end,1));
    title('first comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BI2BO}(1)');
    subplot(2,2,2);
    plot(time(end-5000:end),q_BI2BO(end-5000:end,2));
    title('second comp. ') ;
    xlabel('time in sec');
    %ylabel('q_{BI2BO}(2)');
    subplot(2,2,3);
    plot(time(end-5000:end),q_BI2BO(end-5000:end,3));
    title('third comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BI2BO}(3)');
    xlabel('time in sec');
    subplot(2,2,4);
    plot(time(end-5000:end),q_BI2BO(end-5000:end,4));
    title('fourth comp.') ;
    xlabel('time in sec');
    % ylabel('q_{BI2BO}(4)');
    legend({'$q_{BO}$','$q_{BI2BO}$'},'Interpreter','latex','Location','best');
    %---------------------------------------------------------
    figure(10)
    plot(time,del_theta);
    xlabel('time in sec');
    ylabel('angle in radians');
    legend({'$\delta\theta_{q_{BO}, q_{BI2BO}}$'},'Interpreter','latex','Location','best');
    title({'Rotation angle of the error quaterion between $q_{BO}$  and $q_{BI2BO}$'},'Interpreter','latex');
    save results_wrong_uncont_dist_ident.mat q_BO q_BI q_BI2BO 
 elseif k==3
    in1=0;
    in2=0;
    in3=150;
    in4=0.1;
    t1=0; % initial time
    t2=in3; % final time for the typical integration
    h=in4; % time step
    wd=[0;beta;0];
    kp=0.0001;
    kv=0.0001;
    disp('The angular velocity w_OIO is ') 
    wd
    if in5==1
        I=0.001*eye(3);% declaring the moment of inertia
    else     
        Ixx = 0.00152529;
        Iyy = 0.00145111;
        Izz = 0.001476;
        Ixy = 0.00000437;
        Iyz = - 0.00000408;
        Ixz = 0.00000118;
        I= [Ixx, Ixy, Ixz; Ixy, Iyy, Iyz; Ixz, Iyz, Izz];	
    end 
    qBI0=(1/(2))*[1,1,1,1]'  % initial quaternion 
    qOI0=q_orbit(t1)
    qBO0=quat_prod_onenorm(qBI0,quat_inv(qOI0))
    wBIB0=1e-3*[0,-1,1]' % initial omega
    wBOB0=wBIB0-quat2Rot(qBO0)*wd
    %%Integration 

    if in1==1 && in2==1 % uncontrolled and external torque
        F=@DynamicsBI;
        G=@DynamicsBO;
    elseif in1==1 && in2==0  %uncontrolled and no external torque
        F=@Dynamics_uncontrolledBI; 
        G=@Dynamics_uncontrolledBO;
    elseif  in1==0 && in2==0 % controlled and no external torque
        F=@DynamicsBI;
        G=@DynamicsBO;
    else
        disp('Not a valid choice of inputs')
        pause(5)
        exit
    end 
    % Typical Rk4 integrationin BI
    [time, state]=rk4(F,h,t1,t2,[qBI0;wBIB0]);
    %Typical RK4 integration in BO
    [time2, state2]=rk4(G,h,t1,t2,[qBO0;wBOB0]);
    N=length(time);
    q_true=zeros(N,4);
    q_BI=state(:,1:4);
    q_BO=state2(:,1:4);
    q_BI2BO=zeros(N,4);
    w_BIB=state(:,5:7);
    w_BOB=state2(:,5:7);
    w_BIB2BOB=zeros(N,3);
    q_err=zeros(N,4);
    del_theta=zeros(N,1);
    for i=1:N
        q_BI2BO(i,:)=quat_prod_onenorm(q_BI(i,:)',quat_inv(q_orbit(time(i,1))))';
        w_BIB2BOB(i,:)=(w_BIB(i,:)'-quat2Rot(q_BO(i,:)')*wd)';
        q_err(i,:)=quat_prod_onenorm(quat_inv(q_BI2BO(i,:)'),q_BO(i,:)')';
        del_theta(i)=2*acos(abs(q_err(i,4)));
    end
    %% plotting the results
    setGraphics;
    %------------------------------------------
    %q_BO=q_BO-q_BI2BO;
    figure(11);
    subplot(2,2,1);
    plot(time,q_BO(:,1));
    title('first comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BO}(1)');
    hold on;
    subplot(2,2,2);
    plot(time,q_BO(:,2));
    title('second comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BO}(2)');
    hold on;
    subplot(2,2,3);
    plot(time,q_BO(:,3));
    title('third comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BO}(3)');
    hold on;
    subplot(2,2,4);
    plot(time,q_BO(:,4));
    title('fourth comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BO}(4)');
    hold on;
    %print(gcf,'quat_err_rk4.pdf','-dpdf','-bestfit','-r400');
    % figure(4);
    subplot(2,2,1);
    plot(time,q_BI2BO(:,1));
    title('first comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BI2BO}(1)');
    subplot(2,2,2);
    plot(time,q_BI2BO(:,2));
    title('second comp. ') ;
    xlabel('time in sec');
    %ylabel('q_{BI2BO}(2)');
    subplot(2,2,3);
    plot(time,q_BI2BO(:,3));
    title('third comp.') ;
    xlabel('time in sec');
    %ylabel('q_{BI2BO}(3)');
    xlabel('time in sec');
    subplot(2,2,4);
    plot(time,q_BI2BO(:,4));
    title('fourth comp.') ;
    xlabel('time in sec');
    % ylabel('q_{BI2BO}(4)');
    legend({'$q_{BO}$','$q_{BI2BO}$'},'Interpreter','latex','Location','best');
    %---------------------------------------------------------
    figure(12)
    plot(time,del_theta);
    xlabel('time in sec');
    ylabel('angle in radians');
    legend({'$\delta\theta_{q_{BO}, q_{BI2BO}}$'},'Interpreter','latex','Location','best');
    title({'Rotation angle of the error quaterion between $q_{BO}$  and $q_{BI2BO}$'},'Interpreter','latex');
    save results_wrong_cont_nodist_ident.mat q_BO q_BI q_BI2BO 
 end 
end 