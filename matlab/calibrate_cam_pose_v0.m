
H_b0_w = [-1 0 0 0;
            0 -1 0 0;
            0 0 1 0;
            0 0 0 1];
H_b0_c = [-0.027  0.998  -0.055    0.120;
0.592  -0.029  -0.805    0.131;
-0.805  -0.054  -0.590    1.871;
            0 0 0 1];




H_c_w = H_b0_w*inv(H_b0_c);

q_c_w = rotm2quat(H_c_w(1:3,1:3));
p_c_w = H_c_w(1:3,4);


fprintf('For tf\n')
fprintf('p_c_w = [%.4f %.4f %.4f]\n',p_c_w)
fprintf('q_c_w = [%.4f %.4f %.4f %.4f]\n',q_c_w(2),q_c_w(3),q_c_w(4),q_c_w(1))
fprintf('[%.4f %.4f %.4f %.4f %.4f %.4f %.4f]\n',p_c_w(1),p_c_w(2),p_c_w(3),q_c_w(2),q_c_w(3),q_c_w(4),q_c_w(1))

fprintf('\nFor H_c_w\n')
fprintf('[[%.4f, %.4f, %.4f, %.4f], ', H_c_w(1,:))
fprintf('[%.4f, %.4f, %.4f, %.4f], ', H_c_w(2,:))
fprintf('[%.4f, %.4f, %.4f, %.4f], ', H_c_w(3,:))
fprintf('[%.4f, %.4f, %.4f, %.4f]]\n ', H_c_w(4,:))



% M = [0 1 0;
%     1 0 0;
%     0 0 -1]