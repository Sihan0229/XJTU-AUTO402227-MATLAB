% 三连杆机械臂瞬态运动的牛顿-欧拉递归逆动力学求解：
% 参数：（运动指令）各关节运动角度， 关节速度， 关节加速度（3*1矩阵）
% 返回值：各关节力矩（3*1矩阵）
%theta=[10;20;30];
%theta_d=[1;2;3];
%theta_dd=[0.5;1;1.5];
function tau = three_dof_reverse_dynamics(theta, theta_d, theta_dd)
% 改进D-H参数
th(1) = theta(1)*pi/180; d(1) = 0; a(1) = 0; alp(1) = 0;
th(2) = theta(2)*pi/180; d(2) = 0; a(2) = 4; alp(2) = 0;
th(3) = theta(3)*pi/180; d(3) = 0; a(3) = 3; alp(3) = 0;
% base_link的各项初始值
w00 = [0; 0; 0]; v00 = [0; 0; 0]; w00d = [0; 0; 0]; v00d = [0; 9.8; 0];
% 各关节p及各link质心pc的距离(假设质心在几何中心)
p10 = [0; 0; 0]; p21 = [4; 0; 0];
p32 = [3; 0; 0]; p43 = [2; 0; 0];
pc11 = [2; 0; 0]; pc22 = [1.5; 0; 0];
pc33 = [1; 0; 0];
z = [0; 0; 1];
% 各连杆质量
m1 = 20; m2 = 15; m3 = 10;
% 惯性张量
I1 = [0 0 0; 0 0 0; 0 0 0.5]; I2 = [0 0 0; 0 0 0; 0 0 0.2]; I3 = [0 0 0; 0 0 0; 0 0 0.1];
%% 旋转矩阵T的求解
%关节0到关节1转换矩阵的求解
MDH = [ alp(1) a(1) d(1) th(1);
alp(2) a(2) d(2) th(2);
alp(3) a(3) d(3) th(3)];MH0_1 = [cos(MDH(1,4)) -sin(MDH(1,4)) 0 MDH(1,2);
cos(MDH(1,1))*sin(MDH(1,4)) cos(MDH(1,1))*cos(MDH(1,4)) - sin(MDH(1,1)) -MDH(1,3)*sin(MDH(1,1));
sin(MDH(1,1))*sin(MDH(1,4)) sin(MDH(1,1))*cos(MDH(1,4)) cos(MDH(1,1)) MDH(1,3)*cos(MDH(1,1));
0 0 0 1];
T01 = MH0_1;
%T01 = simplify(MH0_1);
%关节1到关节2转换矩阵的求解
MH1_2 = [cos(MDH(2,4)) -sin(MDH(2,4)) 0
MDH(2,2);
cos(MDH(2,1))*sin(MDH(2,4)) cos(MDH(2,1))*cos(MDH(2,4)) - sin(MDH(2,1)) -MDH(2,3)*sin(MDH(2,1));
sin(MDH(2,1))*sin(MDH(2,4)) sin(MDH(2,1))*cos(MDH(2,4)) cos(MDH(2,1)) MDH(2,3)*cos(MDH(2,1));
0 0 0 1];
T12 = MH1_2;
%T02 = simplify(MH1_2);
%关节2到关节3转换矩阵的求解
MH2_3 = [cos(MDH(3,4)) -sin(MDH(3,4)) 0
MDH(3,2);
cos(MDH(3,1))*sin(MDH(3,4)) cos(MDH(3,1))*cos(MDH(3,4)) - sin(MDH(3,1)) -MDH(3,3)*sin(MDH(3,1));
sin(MDH(3,1))*sin(MDH(3,4)) sin(MDH(3,1))*cos(MDH(3,4)) cos(MDH(3,1)) MDH(3,3)*cos(MDH(3,1));
0 0 0 1];
T23 = MH2_3;
%T03 = simplify(MH2_3);
R01 = T01(1:3, 1:3);
R12 = T12(1:3, 1:3);
R23 = T23(1:3, 1:3);
R10 = R01';
R21 = R12';
R32 = R23';
R34 = [1 0 0; 0 1 0; 0 0 1];
%% Outward iterations: i: 0->2
%运动外推：计算角度、角速度、角加速度
%力外推：计算作用在连杆质心上的惯性力和力矩
% i = 0 连杆1
w11 = R10*w00 + theta_d(1)*z;
w11d = R10*w00d + cross(R10*w00, z*theta_d(1)) + theta_dd(1)*z;
v11d = R10*(cross(w00d, p10) + cross(w00, cross(w00, p10)) + v00d);
vc11d = cross(w11d, pc11) + cross(w11, cross(w11, pc11)) + v11d;
F11 = m1*vc11d;
N11 = I1*w11d + cross(w11, I1*w11);
% i = 1 连杆2
w22 = R21*w11 + theta_d(2)*z;
w22d = R21*w11d + cross(R21*w11, z*theta_d(2)) + theta_dd(2)*z;
v22d = R21*(cross(w11d, p21) + cross(w11, cross(w11, p21)) + v11d);
vc22d = cross(w22d, pc22) + cross(w22, cross(w22, pc22)) + v22d;
F22 = m2*vc22d;
N22 = I2*w22d + cross(w22, I2*w22);
% i = 2 连杆3
w33 = R32*w22 + theta_d(3)*z;
w33d = R32*w22d + cross(R32*w22, z*theta_d(3)) + theta_dd(3)*z;
v33d = R32*(cross(w22d, p32) + cross(w22, cross(w22, p32)) + v22d);
vc33d = cross(w33d, pc33) + cross(w33, cross(w33, pc33)) + v33d;
F33 = m3*vc33d;
N33 = I3*w33d + cross(w33, I3*w33);
%% Inward iterations: i: 3->1
%力矩内推：向内迭代计算关节力矩
f44 = [0; 0; 0]; n44 = [0; 0; 0];
% i = 3
f33 = R34*f44 + F33;
n33 = N33 + R34*n44 + cross(pc33, F33) + cross(p43, R34*f44);
tau(3) = n33'*z;
% i = 2
f22 = R23*f33 + F22;
n22 = N22 + R23*n33 + cross(pc22, F22) + cross(p32, R23*f33);
tau(2) = n22'*z;
% i =1
f11 = R12*f22 + F11;
n11 = N11 + R12*n22 + cross(pc11, F11) + cross(p21, R12*f22);
tau(1) = n11'*z;
tau
end