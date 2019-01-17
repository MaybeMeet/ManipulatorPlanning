%% Overview of our manipulator
clear
load lim.mat
load T
L1=RevoluteMDH('alpha',0,    'a',0,'d',13,'qlim',lim1);
L2=RevoluteMDH('alpha',-pi/2,'a',0,'d',0,'qlim',lim2);
L3=RevoluteMDH('alpha',0,    'a',8,'d',2,'qlim',lim3);
L4=RevoluteMDH('alpha',-pi/2,'a',0,'d',8,'qlim',lim4);
L5=RevoluteMDH('alpha',pi/2  ,'a',0,'d',0,'qlim',lim5);
L6=RevoluteMDH('alpha',pi/2,  'a',0,'d',0,'qlim',lim6);
L7=RevoluteMDH('alpha',0,     'a',0,'d',2,'qlim',[0,0]);
robot=SerialLink([L1,L2,L3,L4,L5,L6,L7]);
robot.display();
theta=[0,pi/6,-3*pi/4,0,3*pi/4,0];
theta=[theta,0];
robot.plot(theta,'jointdiam',1,'arrow','jaxes','wrist'); 	%显示机器人的图像
theta=theta/pi*180;
T=Joint2Pose(T{7},theta(1),theta(2),theta(3),theta(4),theta(5),theta(6));