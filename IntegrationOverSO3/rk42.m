function [time, y]=rk42(F,h,t1,t2,y0)
% this integrator is used for the new interation technique which integrates
% the euler equation using RK4 and the quaternion using first order
% approximation of the exponential map 
% this function calls Dynamics2.m
N=ceil((t2-t1)/h); % No of integration step 
time=zeros(N,1); % initialising time 
time(1,1)=t1;   
 
y=zeros(3*N,4); % initialisng state
y(1:3, :)=y0;  
for i=1:N-1
    t1=time(i); % currrent time
    z=y(3*i-2:3*i,:); % current total state
    R1=z(:,1:3); % current quaternion
    w1=z(:,end); % current angular velocity 
    % only integrating euler's equation to get the agular velocity at next
    % instant using typical RK4 routine 
    p1=h*F(t1,(w1));
    p2=h*F(t1+h/2,(w1+p1/2));
    p3=h*F(t1+h/2,(w1+p2/2));
    p4=h*F(t1+h,(w1+p3));
    w2=(w1+(1/6)*(p1+2*p2+2*p3+p4)); % angular velocity at next instant
    % propogating quaternion using exponential map calculated by assuming
    % constant anguar velocity w1  
    q2=[sin(0.5*norm(w1)*h)*w1/norm(w1);cos(0.5*norm(w1)*h)]; 
    R2=quat2Rot(q2);
    R=brute_force_orth(R2*R1);%
    % concatenating both of them into a single vector
    y(3*(i+1)-2:3*(i+1),:)=[R,w2];
    time(i+1)=t1+h; % time for the next instant 
end
