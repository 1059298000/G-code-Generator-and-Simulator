function [O,Q] = forKinPlot(joint_params,offset,pose,prev_pose,prev_R,axis_vec,arrow_length,count)

joint_params = gpuArray(joint_params);
num_steps = 50;
joint_params_steps = gpuArray(zeros([5,num_steps+1]));
O = gpuArray(zeros([3,num_steps+1]));
Q = gpuArray(zeros([3,num_steps+1]));

for i = 1:num_steps+1
    joint_params_steps(:,i) = [(i-1).*joint_params.'./num_steps];
    
    O(:,i) = [sin(joint_params_steps(5,i))*sin(joint_params_steps(4,i));
        -cos(joint_params_steps(5,i))*sin(joint_params_steps(4,i));
        cos(joint_params_steps(4,i))];
    
    Q(:,i) = [cos(joint_params_steps(5,i))*joint_params_steps(1,i)-sin(joint_params_steps(5,i))*cos(joint_params_steps(4,i))*joint_params_steps(2,i)+sin(joint_params_steps(5,i))*sin(joint_params_steps(4,i))*joint_params_steps(3,i)+offset(1);
    sin(joint_params_steps(5,i))*joint_params_steps(1,i)+cos(joint_params_steps(5,i))*cos(joint_params_steps(4,i))*joint_params_steps(2,i)-cos(joint_params_steps(5,i))*sin(joint_params_steps(4,i))*joint_params_steps(3,i)+offset(2);
    sin(joint_params_steps(4,i))*joint_params_steps(2,i)+cos(joint_params_steps(4,i))*joint_params_steps(3,i)+offset(3)];
    
end

O = gather(prev_R*O);
Q = gather(Q);
plot3(Q(1,:),Q(2,:),Q(3,:),'b');
hold on;
plot3([prev_pose(1),pose(1)],[prev_pose(2),pose(2)],[prev_pose(3),pose(3)]);
hsv(1,1,1) = 0.3; % hue
hsv(1,1,2) = 1; % saturation
hsv(1,1,3) = 1; % value
for i=1:num_steps+1
    rgb = hsv2rgb(hsv);
    if mod(i,count)==0 || i==1 || i==num_steps+1
        mArrow3(Q(:,i),Q(:,i)+arrow_length*O(:,i),'stemWidth',arrow_length*3/500,'color',rgb);
    end
    hsv(1,1,1) = hsv(1,1,1) + 0.005;
end
hold off;
xlabel("X"),ylabel("Y"),zlabel("Z");
axis(axis_vec);
grid on;
end

