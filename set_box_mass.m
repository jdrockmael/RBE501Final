%% Function laods urdf with box and sets mass
% robot = importrobot('irb1600id_box.urdf','DataFormat','column');
% robot_m1 = set_box_mass(robot, 1); % Set  box mass to 1 kg

% Robot loads 

function robot = set_box_mass(robot, m)
    ixx = m/6 * (0.2^2);
    robot.Bodies{1,8}.Mass = m;
    robot.Bodies{1,8}.Inertia = [ixx, ixx, ixx, 0, 0, 0];
end