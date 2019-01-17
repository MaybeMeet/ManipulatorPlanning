%% Display a Solution
clear
load e1.mat
load lim.mat
load T.mat
 %9.3711   10.4675   -4.9505
L1=RevoluteMDH('alpha',0,    'a',0,'d',0,'qlim',lim1);
L2=RevoluteMDH('alpha',-pi/2,'a',0,'d',0,'qlim',lim2);
L3=RevoluteMDH('alpha',0,    'a',8,'d',2,'qlim',lim3);
L4=RevoluteMDH('alpha',-pi/2,'a',0,'d',8,'qlim',lim4);
L5=RevoluteMDH('alpha',pi/2  ,'a',0,'d',0,'qlim',lim5);
L6=RevoluteMDH('alpha',pi/2,  'a',0,'d',0,'qlim',lim6);
L7=RevoluteMDH('alpha',0,     'a',0,'d',2,'qlim',[0,0]);

robot=SerialLink([L1,L2,L3,L4,L5,L6,L7]);
theta0=Theta_Planned(1,:);
pose=Joint2Pose(T{7},theta0(1),theta0(2),...
    theta0(3),theta0(4),theta0(5),theta0(6));
Theta_Planned=[Theta_Planned,zeros([length(Theta_Planned(:,1)),1])];
Theta_Planned=Theta_Planned*pi/180;
while 1
    figure(1)
    plot3(pose(4),pose(5),pose(6),'*','LineWidth',1.5);
    robot.plot(Theta_Planned,'jointdiam',1,'arrow','jaxes','wrist');
end