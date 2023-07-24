clear; clc;

irb1600id = importrobot('irb1600id.urdf','DataFormat','column');
irb1600id.Gravity = [0, 0, -9.8];

% jointInitialPos_Vel = [0,0,0,0,pi/3,0,0,0,0,0,0,0]';                       % define initial joint angles to be [0,0,0,0,0,0,0,0,0,0,0,0]
% jointTargetPos = [pi/6, pi/6, pi/6, 0, pi/2, 0]';                          % define desired joint angles. 
% jointTargetVel = [0, 0, 0, 0, 0, 0]';

inputvar.pos_0 = [0, pi/2, 0, 0, 0, 0];
inputvar.vel_0 = [0, 0, 0, 0, 0, 0];
inputvar.pos_end = [0, 0, 0, 0, 0, 0];
inputvar.vel_end = [0, 0, 0, 0, 0, 0];

inputvar.PID.P = 50;
inputvar.PID.I = 0;
inputvar.PID.D = 0;

sim_values = sim("irb1600.slx");

pos_out = sim_values.yout{1}.Values.Data;

%% animation
figure()                                                                   % create new figure and set figure properties
set(gcf,'Visible','on');
show(irb1600id, pos_out(1,:)');                                                  % show robot initial joint configuration from state space
view(60,10);                                                               % set 3D view (azimuth & elevation angle)
hold on
interval = round(0.01*length(pos_out));                                          % set animation update interval (we have too many states)
for i = 1:interval:length(pos_out)
    jointPos = pos_out(i,1:6);                                                 % get current joint positions from state space
    show(irb1600id,jointPos','PreservePlot',false);                            % show robot at current joint configuration
    title(sprintf('Frame = %d of %d', i, length(pos_out)));                      % set figure title
    xlim([-1,1]); ylim([-1,1]); zlim([0,2]);                               % limitaxis range
    drawnow                                                                % forceanimation to update
end