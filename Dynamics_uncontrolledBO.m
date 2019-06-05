function xdot=Dynamics_uncontrolledBO(t,x)
%this function is used for normal RK4 integration 
%this function is called by rk4.m
global I beta in2; % moment of inertia matrix declared in main code
q=x(1:4,1); % extracting quaternion 
delw=x(5:7,1); % extracting omega
wd=[0;beta;0];
q_dot=0.5*quat_prod([delw;0],q); % calculating the derivative of quaternion
R=quat2Rot(q);
w=delw+R*wd;
if in2==1
delw_dot=I\(torque(t)-cross(w,I*w)-I*(cross((R)*wd,delw))); % calculating the derivative of omega, correct
%---------------
% incorrect equation 
%delw_dot=I\(torque(t)-cross(w,I*w)+I*(R)*(cross(wd,delw))); 
%-----------------
else 
    delw_dot=I\(-cross(w,I*w)-I*(cross((R)*wd,delw)));
   %--------------------
   % incorrect equation 
   % delw_dot=I\(-cross(w,I*w)+I*(R)*(cross(wd,delw)));
   %--------------
end 
xdot=[q_dot; delw_dot]; % concatenating both the derivatives 

