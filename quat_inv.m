function q=quat_inv(p)
q=(1/(norm(p))^2)*[-p(1:3,1);p(4)]; 
