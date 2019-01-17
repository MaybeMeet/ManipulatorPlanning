%% 演示多解的情况
load lim.mat
theta=[54,27,-135,-36,144,-36];
pose=Joint2Pose(theta(1),theta(2),theta(3),theta(4),theta(5),theta(6));
param=InvK(pose(1),pose(2),pose(3),pose(4),pose(5),pose(6));
L1=RevoluteMDH('alpha',0,    'a',0,'d',0,'qlim',lim1);
L2=RevoluteMDH('alpha',-pi/2,'a',0,'d',0,'qlim',lim2);
L3=RevoluteMDH('alpha',0,    'a',8,'d',2,'qlim',lim3);
L4=RevoluteMDH('alpha',-pi/2,'a',0,'d',8,'qlim',lim4);
L5=RevoluteMDH('alpha',pi/2  ,'a',0,'d',0,'qlim',lim5);
L6=RevoluteMDH('alpha',pi/2,  'a',0,'d',0,'qlim',lim6);
L7=RevoluteMDH('alpha',0,     'a',0,'d',2,'qlim',[0,0]);
robot=SerialLink([L1,L2,L3,L4,L5,L6,L7]);
robot.plot([param(1,:)*pi/180,0],'jointdiam',1,'arrow','jaxes','wrist');
pause
robot.plot([param(2,:)*pi/180,0],'jointdiam',1,'arrow','jaxes','wrist');
