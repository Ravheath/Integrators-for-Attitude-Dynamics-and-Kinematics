function xdot=DynamicsBI(t,x)
%this function is used for normal RK4 integration 
%this function is called by rk4.m
global I in1 wd kp kv; % moment of inertia matrix declared in main code
q=x(1:4); % extracting quaternion 
w=x(5:7); % extracting omega
q_dot=0.5*quat_prod([w;0],q); % calculating the derivative of quaternion
if in1==1
    w_dot=I\(torque(t)-cross(w,I*w)); % calculating the derivative of omega
else 
    delq=quat_prod_onenorm(q,quat_inv(q_orbit(t)));
    R=quat2Rot(delq);
    delw=w-R*wd;
       tau=cross(w,I*w)+I*(cross((R)*wd,delw))-kp*delq(1:3)-kv*delw; % correct
        %-------------------
        %incorrect equation 
       %tau=cross(w,I*w)-I*(R)*(cross(wd,delw))-kp*delq(1:3)-kv*delw;
       %--------------------------
    w_dot=I\(tau-cross(w,I*w));
end 
xdot=[q_dot; w_dot]; % concatenating both the derivatives 

