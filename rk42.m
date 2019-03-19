function [time, y]=rk42(F,h,t1,t2,y0)
% this integrator is used for the new interation technique which integrates
% the euler equation using RK4 and the quaternion using first order
% approximation of the exponential map 
% this function calls Dynamics2.m
N=ceil((t2-t1)/h); % No of integration step 
time=zeros(N,1); % initialising time 
time(1,1)=t1;   
M=length(y0); 
y=zeros(N,M); % initialisng state
y(1,:)=y0';  
for i=1:N-1
    t1=time(i); % currrent time
    z=y(i,:); % current total state
    q1=z(1:4); % current quaternion
    w1=z(5:7); % current angular velocity 
    % only integrating euler's equation to get the agular velocity at next
    % instant using typical RK4 routine 
    p1=h*F(t1,(w1'));
    p2=h*F(t1+h/2,(w1'+p1/2));
    p3=h*F(t1+h/2,(w1'+p2/2));
    p4=h*F(t1+h,(w1'+p3));
    w2=(w1'+(1/6)*(p1+2*p2+2*p3+p4))'; % angular velocity at next instant
    % propogating quaternion using exponential map calculated by assuming
    % constant anguar velocity w1  
    q2=[sin(0.5*norm(w1)*h)*w1'/norm(w1);cos(0.5*norm(w1)*h)]; 
    q2=quat_prod(q1',q2);
    % concatenating both of them into a single vector
    y(i+1,:)=[q2',w2];
    time(i+1)=t1+h; % time for the next instant 
end
