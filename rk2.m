function [time, y]=rk2(F,h,t1,t2,y0)
% this integrator is used for the typical integration technique which integrates
% the euler equation and the kinematics equation using RK4 and then brute force normalising the quaternion 
% this function calls Dynamics.m
N=ceil((t2-t1)/h); % number of integration steps
time=zeros(N,1); 
time(1,1)=t1;
M=length(y0);
y=zeros(N,M);
y(1,:)=y0';
for i=1:N-1
    t1=time(i); % current time
    % typical RK4 routine
    p1=h*F(t1,(y(i,:)')); 
    p2=h*F(t1+h/2,(y(i,:)'+p1/2));
    y1=(y(i,:)'+(1/2)*(p1+p2))';
    z=y1(1:4); % extracting the quaternion 
    z=z/(norm(z)); % normalising the quaternion
    y(i+1,:)=[z,y1(5:7)]; % concatenating both the quaternion and omega into a vector
    time(i+1)=t1+h; % next time instant
end