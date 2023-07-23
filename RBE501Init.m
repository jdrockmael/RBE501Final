%% simulation
clc; clear; close all
global robot dof jointTargetPos jointTargetVel 
robot = importrobot('irb1600id.urdf','DataFormat','column');               % load robot model, set data format to column, set gravity vector
robot.Gravity = [0 0 -9.8];
planner = Traj_Planner();
dof = numel(homeConfiguration(robot));                                     % get robot degree of freedom
jointInitialPos_Vel = [0,0,0,0,pi/6,0,0,0,0,0,0,0]';                       % define initial joint angles to be [0,0,0,0,0,0,0,0,0,0,0,0]
jointTargetPos = [pi/6, pi/6, pi/6, 0, 0.33, 0]';                          % define desired joint angles. 
jointTargetVel = [0, 0, 0, 0, 0, 0]';
Tf = 0.6;                                                                  % simulation end time
tSpan = [0, Tf];                                                           % define simulation time span
tic;                                                                       % benchmarking
[T, X] = ode45(@(t,x)armODE(t,x),tSpan,jointInitialPos_Vel);               % solve robot dynamical model dq=F(q,dq), robot state space is defined as X=[q, dq]
toc;

traj_Mat1 = planner.quintic_traj(0, Tf, jointInitialPos_Vel(1,:),...       % trajectory planning function at each joint
    jointTargetPos(1), jointInitialPos_Vel(7,:), jointTargetVel(1), 0, 0); % cubic trajectory can also be used, but left quintic in case we want to define acceleration
traj_Mat2 = planner.quintic_traj(0, Tf, jointInitialPos_Vel(2,:),...
    jointTargetPos(2), jointInitialPos_Vel(8,:), jointTargetVel(2), 0, 0);
traj_Mat3 = planner.quintic_traj(0, Tf, jointInitialPos_Vel(3,:),...
    jointTargetPos(3), jointInitialPos_Vel(9,:), jointTargetVel(3), 0, 0);
traj_Mat4 = planner.quintic_traj(0, Tf, jointInitialPos_Vel(4,:),...
    jointTargetPos(4), jointInitialPos_Vel(10,:), jointTargetVel(4), 0, 0);
traj_Mat5 = planner.quintic_traj(0, Tf, jointInitialPos_Vel(5,:),...
    jointTargetPos(5), jointInitialPos_Vel(11,:), jointTargetVel(5), 0, 0);
traj_Mat6 = planner.quintic_traj(0, Tf, jointInitialPos_Vel(6,:),...
    jointTargetPos(6), jointInitialPos_Vel(12,:), jointTargetVel(6), 0, 0);

traj_Mat = [traj_Mat1,traj_Mat2,traj_Mat3,traj_Mat4,traj_Mat5,traj_Mat6];  % combine joints

%% animation
figure()                                                                   % create new figure and set figure properties
set(gcf,'Visible','on');
show(robot, X(1,1:dof)');                                                  % show robot initial joint configuration from state space
view(60,10);                                                               % set 3D view (azimuth & elevation angle)
hold on
interval = round(0.01*length(X));                                          % set animation update interval (we have too many states)
for i = 1:interval:length(X)
    %jointPos = traj_Mat(i,1:dof);
    jointPos = X(i,1:dof);                                                  % get current joint positions from state space
    show(robot,jointPos','PreservePlot',false);                            % show robot at current joint configuration
    title(sprintf('Frame = %d of %d', i, length(X)));                      % set figure title
    xlim([-1,1]); ylim([-1,1]); zlim([0,2]);                               % limitaxis range
    drawnow                                                                % forceanimation to update
end

%% Plot
figure()
for i = 1:dof
    hold on
    plot(T, X(:, i), 'LineWidth', 1);
end
hold off
xlabel('time [sec]');
ylabel('joint angle [rad');
grid on
legend('q1', 'q2', 'q3', 'q4','q5','q6');

figure()
for i = dof+1:2*dof
    hold on
    plot(T, X(:, i), 'LineWidth', 1);
end
hold off
xlabel('time [sec]');
ylabel('joint velocity [rad/s]');
grid on
legend('q1''', 'q2''', 'q3''', 'q4''','q5''','q6''');

figure()
for i = 7:12
    hold on
    plot(T(2:end,:), diff(X(:, i)), 'LineWidth', 1);
end
hold off
xlabel('time [sec]');
ylabel('joint acceleration [rad/s^2]');
grid on
legend('q1"', 'q2"', 'q3"', 'q4"','q5"','q6"');

%% utilities
function dx = armODE(~, x)
global jointTargetPos jointTargetVel robot dof    
    %tau = zeros(6,1); % without controller
    tau = jointPD(jointTargetPos, jointTargetVel, x);% PID-controller
    dx = zeros(dof*2, 1);
    dx(1:6) = x(7:12);
    dx(dof+1:end) = forwardDynamics(robot, x(1:dof),x(dof+1:end),tau,[]);  
end

function tau = jointPD(joint_target_pos,joint_target_vel,x)
   Kp = 25000;
   Ki = 10000;
   Kd = 3500;
   t1 = (joint_target_pos(1)-x(1))*Kp + Ki*cumtrapz((joint_target_pos(1)-x(1)),1) + (joint_target_vel(1)-x(7))*Kd;
   t2 = (joint_target_pos(2)-x(2))*Kp + Ki*cumtrapz((joint_target_pos(2)-x(2)),1) + (joint_target_vel(2)-x(8))*Kd;
   t3 = (joint_target_pos(3)-x(3))*Kp + Ki*cumtrapz((joint_target_pos(3)-x(3)),1) + (joint_target_vel(3)-x(9))*Kd;
   t4 = (joint_target_pos(4)-x(4))*Kp + Ki*cumtrapz((joint_target_pos(4)-x(4)),1) + (joint_target_vel(4)-x(10))*Kd;
   t5 = (joint_target_pos(5)-x(5))*Kp + Ki*cumtrapz((joint_target_pos(5)-x(5)),1) + (joint_target_vel(5)-x(11))*Kd;
   t6 = (joint_target_pos(6)-x(6))*Kp + Ki*cumtrapz((joint_target_pos(6)-x(6)),1) + (joint_target_vel(6)-x(12))*Kd;
   tau = [t1 t2 t3 t4 t5 t6];
   tau = tau';
end