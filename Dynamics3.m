function xdot=Dynamics3(t,x)
%this function is used for normal RK4 integration 
%this function is called by rk4.m
global I beta; % moment of inertia matrix declared in main code
q=x(1:4,1); % extracting quaternion 
delw=x(5:7,1); % extracting omega
wd=[0;beta;0];
q_dot=0.5*quat_prod([delw;0],q); % calculating the derivative of quaternion
R=quat2Rot(q);
w=delw+R*wd;
delw_dot=I\(torque(t)-cross(w,I*w)-I*(cross((R)*wd,delw))); % calculating the derivative of omega
xdot=[q_dot; delw_dot]; % concatenating both the derivatives 

