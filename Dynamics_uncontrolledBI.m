function xdot=Dynamics_uncontrolledBI(t,x)
%this function is used for normal RK4 integration 
%this function is called by rk4.m
global I in2; % moment of inertia matrix declared in main code
q=x(1:4); % extracting quaternion 
w=x(5:7); % extracting omega
q_dot=0.5*quat_prod([w;0],q); % calculating the derivative of quaternion
if in2==1
w_dot=I\(torque(t)-cross(w,I*w)); % calculating the derivative of omega
else 
 w_dot=I\(-cross(w,I*w));
end 
xdot=[q_dot; w_dot]; % concatenating both the derivatives 

