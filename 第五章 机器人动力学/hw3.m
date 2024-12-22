% 创建机器人模型
robot = robotics.RigidBodyTree;

% 添加连杆和关节
for i = 1:3
    body = robotics.RigidBody(['link', num2str(i)]);
    joint = robotics.Joint(['joint', num2str(i)], 'revolute');
    setFixedTransform(joint, trvec2tform([L(i), 0, 0]));
    body.Joint = joint;
    addBody(robot, body, robot.BaseName);
end

% 设置机器人状态
config = struct('JointPosition', num2cell(theta), ...
                'JointVelocity', num2cell(theta_dot), ...
                'JointAcceleration', num2cell(theta_ddot));

% 验证驱动力矩
tau_verified = inverseDynamics(robot, config);
disp('验证驱动力矩 tau_verified (N·m):');
disp(tau_verified);
