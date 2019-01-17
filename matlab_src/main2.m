%% Config
clear
t=0:1:50;
Rtb=0.3;

%% Init
load lim.mat;
load T.mat;
%theta0=[45,10,-60,45,60,10];
theta0=[10,45,-160,20,120,20];
current=Joint2Pose(theta0(1),theta0(2),...
   theta0(3),theta0(4),theta0(5),theta0(6));
% target=...
%     [current(1)-80,...
%     current(2)+90,...
%     current(3)+70,...
%     current(4:6)];
target=...
    [current(1)-80,...
    current(2)-150,...
    current(3)+80,...
    current(4:6)];
fprintf("Current state [alpha,beta,gamma,x,y,z]:");
disp(current);
fprintf("Target  state [alpha,beta,gamma,x,y,z]:");
disp(target);

L1=RevoluteMDH('alpha',0,    'a',0,'d',0,'qlim',lim1);
L2=RevoluteMDH('alpha',-pi/2,'a',0,'d',0,'qlim',lim2);
L3=RevoluteMDH('alpha',0,    'a',8,'d',2,'qlim',lim3);
L4=RevoluteMDH('alpha',-pi/2,'a',0,'d',8,'qlim',lim4);
L5=RevoluteMDH('alpha',pi/2  ,'a',0,'d',0,'qlim',lim5);
L6=RevoluteMDH('alpha',pi/2,  'a',0,'d',0,'qlim',lim6);
L7=RevoluteMDH('alpha',0,     'a',0,'d',2,'qlim',[0,0]);
robot=SerialLink([L1,L2,L3,L4,L5,L6,L7]);

figure(1);
theta1=theta0*pi/180;
robot.plot([theta1,0],'jointdiam',1,'arrow','jaxes','wrist');
title("current");
param = InvK(target(1),target(2),...
     target(3),target(4),target(5),target(6));
theta2=param(1,:);
dtheta=(theta2-theta0)*(theta2-theta0)';
for idx = 1:1:length(param(:,1))
    if (param(idx,:)-theta0)*(param(idx,:)-theta0)'<dtheta
        dtheta=(param(idx,:)-theta0)*(param(idx,:)-theta0)';
        theta2=param(idx,:);
    end
end
pause;
robot.plot([theta2*pi/180,0],'jointdiam',1,'arrow','jaxes','wrist');
title("target");
pause
clear theta2 theta1

%% Program

% Plot Catesian Space Planning
figure(2);
alpha=quli(t,Rtb,current(1),target(1));
subplot(3,3,1);
plot(t,alpha,'blue','LineWidth',1.5); grid on;
title("\alpha (t)"); xlabel("t"); ylabel("\alpha");
beta=quli(t,Rtb,current(2),target(2));
subplot(3,3,2);
plot(t,beta,'blue','LineWidth',1.5); grid on;
title("\beta (t)"); xlabel("t"); ylabel("\beta");
gamma=quli(t,Rtb,current(3),target(3));
subplot(3,3,3);
plot(t,gamma,'blue','LineWidth',1.5);grid on;
title("\gamma (t)"); xlabel("t"); ylabel("\gamma");

% Planning of theta
Theta_Planned=theta0;
T_planned=[t(1)];
p=current(4:6);
for index = 1:length(t)
    param=InvK(alpha(index),beta(index),gamma(index),p(1),p(2),p(3));
    if isnan(param(1,1))
        continue;
    end
    theta2=param(1,:);
    theta0=Theta_Planned(end,:);
    dtheta=(theta2-theta0)*(theta2-theta0)';
    for idx = 1:1:length(param(:,1))
        if (param(idx,:)-theta0)*(param(idx,:)-theta0)'<dtheta
            dtheta=(param(idx,:)-theta0)*(param(idx,:)-theta0)';
            theta2=param(idx,:);
        end
    end
    Theta_Planned=[Theta_Planned;theta2];
    T_planned=[T_planned,t(index)];
    fprintf("TimeStampidx: %d Success \n",index);
end

% Plot the results
subplot(3,3,4);
plot(T_planned,Theta_Planned(:,1)','g','LineWidth',1.5); grid on;
title("\theta_1(t)"); xlabel("t"); ylabel("\theta_1");
subplot(3,3,5);
plot(T_planned,Theta_Planned(:,2)','g','LineWidth',1.5); grid on;
title("\theta_2(t)"); xlabel("t"); ylabel("\theta_2");
subplot(3,3,6);
plot(T_planned,Theta_Planned(:,3)','g','LineWidth',1.5); grid on;
title("\theta_3(t)"); xlabel("t"); ylabel("\theta_3");
subplot(3,3,7);
plot(T_planned,Theta_Planned(:,4)','g','LineWidth',1.5); grid on;
title("\theta_4(t)"); xlabel("t"); ylabel("\theta_4");
subplot(3,3,8);
plot(T_planned,Theta_Planned(:,5)','g','LineWidth',1.5); grid on;
title("\theta_5(t)"); xlabel("t"); ylabel("\theta_5");
subplot(3,3,9);
plot(T_planned,Theta_Planned(:,6)','g','LineWidth',1.5); grid on;
title("\theta_6(t)"); xlabel("t"); ylabel("\theta_6");

%% Show

Theta_Planned=[Theta_Planned,zeros([length(Theta_Planned(:,1)),1])];
Theta_Planned=Theta_Planned*pi/180;

figure(1)
plot3(current(4),current(5),current(6),'*','LineWidth',1.5);
robot.plot(Theta_Planned,'jointdiam',1,'arrow','jaxes','wrist');


