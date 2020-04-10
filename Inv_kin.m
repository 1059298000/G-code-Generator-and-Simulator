function [joint_params] = Inv_kin(pose,curr_R,prev_pose,prev_R)

syms g11 g12 g13 g21 g22 g23 g31 g32 g33 real;
syms A C real;
syms Lx Ly Lz Px Py_other Pz real;

offset = prev_pose(1:3);

G = [g11,g12,g13;
     g21,g22,g23;
     g31,g32,g33];
 
invG = inv(G);

out = zeros([1,5]);

invG = subs(invG,'g11',prev_R(1,1));
invG = subs(invG,'g12',prev_R(1,2));
invG = subs(invG,'g13',prev_R(1,3));
invG = subs(invG,'g21',prev_R(2,1));
invG = subs(invG,'g22',prev_R(2,2));
invG = subs(invG,'g23',prev_R(2,3));
invG = subs(invG,'g31',prev_R(3,1));
invG = subs(invG,'g32',prev_R(3,2));
invG = subs(invG,'g33',prev_R(3,3));
    
relative_rot = invG*curr_R;
test = relative_rot;

out(4) = asin(relative_rot(3,2));
out(5) = atan(relative_rot(2,1)/relative_rot(1,1));

out(1) = cos(out(5))*(pose(1)-offset(1)) + sin(out(5))*(pose(2)-offset(2));
out(2) = -sin(out(5))*cos(out(4))*(pose(1)-offset(1)) +...
        cos(out(5))*cos(out(4))*(pose(2)-offset(2)) + ...
        sin(out(4))*(pose(3)-offset(3));
out(3) = sin(out(5))*sin(out(4))*(pose(1)-offset(1)) -...
        cos(out(5))*sin(out(4))*(pose(2)-offset(2)) + ...
        cos(out(4))*(pose(3)-offset(3));
    
joint_params = out;
end

