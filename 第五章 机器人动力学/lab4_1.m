mdl_puma560;
p560.dyn;%查看puma560机械臂所有连杆的动力学参数
p560.dyn(2);%查看puma560机械臂第二连杆的动力学参数
p560.links(2).dyn;%查看puma560机械臂第二连杆的动力学参数

[Tsim, q, qd] = R.fdyn(T, torqfun)


mdl_puma560;
T = transpose([1 1 1 1 1 1]);
theta_0 = transpose([-pi/3 pi/2 pi/6 0 0 0]);
theta_0_prime = transpose([0 0 0 0 0 0]);
% q0 = theta_0;
% qd0 = theta_0_prime;
[T, q, qd] = p560.fdyn(T, torqfun, theta_0, theta_0_prime)
qdd = p560.accel(q, qd, torqfun)

torqfun