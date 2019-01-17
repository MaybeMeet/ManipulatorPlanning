%% Config
clear
tmax=50;
dt=1;
t=0:dt:tmax;
Rtb=0.3;
Amax=5;

%% Init
load lim.mat;
load T.mat;
theta0=[10,45,-160,20,120,20];
current=Joint2Pose(theta0(1),theta0(2),...
   theta0(3),theta0(4),theta0(5),theta0(6));
target=...
    [current(1)-80,...
    current(2)-150,...
    current(3)+80,...
    current(4:6)];
target1=...
    [target(1)-90,...
    target(2)+30,...
    target(3)+80,...
    target(4:6)];
fprintf("Current state [alpha,beta,gamma,x,y,z]:");
disp(current);
fprintf("Target  state [alpha,beta,gamma,x,y,z]:");
disp(target);
fprintf("Target2  state [alpha,beta,gamma,x,y,z]:");
disp(target1);

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
dtheta=delta_theta(theta0,theta2);
for idx = 1:1:length(param(:,1))
    if (param(idx,:)-theta0)*(param(idx,:)-theta0)'<dtheta
        dtheta=(param(idx,:)-theta0)*(param(idx,:)-theta0)';
        theta2=param(idx,:);
    end
end
param = InvK(target1(1),target1(2),...
     target1(3),target1(4),target1(5),target1(6));
theta3=param(1,:);
dtheta=delta_theta(theta0,theta2);
for idx = 1:1:length(param(:,1))
    if (param(idx,:)-theta0)*(param(idx,:)-theta0)'<dtheta
        dtheta=(param(idx,:)-theta0)*(param(idx,:)-theta0)';
        theta3=param(idx,:);
    end
end
%pause;
robot.plot([theta2*pi/180,0],'jointdiam',1,'arrow','jaxes','wrist');
title("target");
%pause
robot.plot([theta3*pi/180,0],'jointdiam',1,'arrow','jaxes','wrist');
title("target2");
%pause
clear theta2 theta1

%% Cartesian Planning and plot
% Plot Cartesian Space Planning
figure(2);
alpha1=quli(t,Rtb,current(1),target(1));
alpha2=quli(t,Rtb,target(1),target1(1));
alpha3=quli(t,Rtb,target1(1),current(1));
alpha=[alpha1,alpha2,alpha3];
subplot(3,3,1);
plot([t,t+tmax,t+2*tmax],alpha,'blue','LineWidth',1.5); grid on;
title("\alpha (t)"); xlabel("t"); ylabel("\alpha");
beta1=quli(t,Rtb,current(2),target(2));
beta2=quli(t,Rtb,target(2),target1(2));
beta3=quli(t,Rtb,target1(2),current(2));
beta=[beta1,beta2,beta3];
subplot(3,3,2);
plot([t,t+tmax,t+2*tmax],beta,'blue','LineWidth',1.5); grid on;
title("\beta (t)"); xlabel("t"); ylabel("\beta");
gamma1=quli(t,Rtb,current(3),target(3));
gamma2=quli(t,Rtb,target(3),target1(3));
gamma3=quli(t,Rtb,target1(3),current(3));
gamma=[gamma1,gamma2,gamma3];
subplot(3,3,3);
plot([t,t+tmax,t+2*tmax],gamma,'blue','LineWidth',1.5);grid on;
title("\gamma (t)"); xlabel("t"); ylabel("\gamma");

%% Planning of theta
Theta_Planned=theta0;
Delta_Theta=[];
t=[t,t+tmax,t+2*tmax];
T_planned=[t(1)];
p=current(4:6);
for index = 1:length(t)
    param=InvK(alpha(index),beta(index),gamma(index),p(1),p(2),p(3));
    if isnan(param(1,1))
        continue;
    end
    theta2=param(1,:);
    theta0=Theta_Planned(end,:);
    dtheta=delta_theta(theta0,theta2);
    for idx = 1:1:length(param(:,1))
        if delta_theta(theta0,theta2)<dtheta
            dtheta=(param(idx,:)-theta0)*(param(idx,:)-theta0)';
            theta2=param(idx,:);
        end
    end
    Delta_Theta=[Delta_Theta;VTheta(theta2,Theta_Planned(end,:))/dt];
    Theta_Planned=[Theta_Planned;theta2];
    T_planned=[T_planned,t(index)];
    
    fprintf("TimeStampidx: %d Success \n",index);
end
% calculate a
A_Theta=(Delta_Theta(2:end,:)-Delta_Theta(1:length(Delta_Theta(:,1))-1,:))./dt;
X_Theta=(Delta_Theta(2:end,:)-Delta_Theta(1:length(Delta_Theta(:,1))-1,:)).*dt;
A_critical=max(abs(X_Theta),[],2);
Dt_critical=sqrt(A_critical./Amax);
% Optimized Dt_critical
newT=Dt_critical./Dt_critical*dt;
ReFlag=0;
for cnt=1:length(Dt_critical)
    if Dt_critical(cnt)>dt
        ReFlag=1;
        ovtime=Dt_critical(cnt)-dt;
        newT(cnt)=Dt_critical(cnt);
        for jdx=1:cnt-1
            if Dt_critical(cnt-jdx)<dt && ovtime>0
                adjust=min(ovtime,(dt-Dt_critical(cnt-jdx))/3);
                ovtime=ovtime-adjust;
                newT(cnt-jdx)=newT(cnt-jdx)-adjust;
            end
            if ovtime <1e-3
                break;
            end
        end
        if ovtime>0
            for jdx=cnt+1:length(Dt_critical)
                if Dt_critical(jdx)<dt
                    adjust=min(ovtime,(dt-Dt_critical(jdx))/3);
                    ovtime=ovtime-adjust;
                    newT(jdx)=newT(jdx)-adjust;
                end
                if ovtime <1e-3
                    break;
                end
            end
        end
    end
end
% If changed replan
if ReFlag>0
    fprintf("Need to adjust timeline\n");
    time_line=[0];
    for cnt=1:length(newT)
        temp=time_line(end)+newT(cnt);
        time_line=[time_line,temp];
    end
    time_line(1)=[];
    V_new=Delta_Theta(1:length(newT),:)./(newT*ones([1,6]));
    A_new=X_Theta./(newT*ones([1,6])).^2;
end

%% Plot the results
figure(2);
subplot(3,3,4);
plot(T_planned,Theta_Planned(:,1)','g','LineWidth',1.5); grid on;
title("\theta_1(t)");  ylabel("\theta_1");
subplot(3,3,5);
plot(T_planned,Theta_Planned(:,2)','g','LineWidth',1.5); grid on;
title("\theta_2(t)");  ylabel("\theta_2");
subplot(3,3,6);
plot(T_planned,Theta_Planned(:,3)','g','LineWidth',1.5); grid on;
title("\theta_3(t)");  ylabel("\theta_3");
subplot(3,3,7);
plot(T_planned,Theta_Planned(:,4)','g','LineWidth',1.5); grid on;
title("\theta_4(t)"); ylabel("\theta_4");
subplot(3,3,8);
plot(T_planned,Theta_Planned(:,5)','g','LineWidth',1.5); grid on;
title("\theta_5(t)"); ylabel("\theta_5");
subplot(3,3,9);
plot(T_planned,Theta_Planned(:,6)','g','LineWidth',1.5); grid on;
title("\theta_6(t)"); ylabel("\theta_6");
figure(3)
% Plot V
subplot(4,3,1);
plot(t,Delta_Theta(:,1)','r','LineWidth',1.5); grid on;
title("D \theta_1(t)");  ylabel("D \theta_1");
subplot(4,3,2);
plot(t,Delta_Theta(:,2)','r','LineWidth',1.5); grid on;
title("D \theta_2(t)"); ylabel("D \theta_2");
subplot(4,3,3);
plot(t,Delta_Theta(:,3)','r','LineWidth',1.5); grid on;
title("D \theta_3(t)"); ylabel("D \theta_3");
subplot(4,3,4);
plot(t,Delta_Theta(:,4)','r','LineWidth',1.5); grid on;
title("D \theta_4(t)"); ylabel("D \theta_4");
subplot(4,3,5);
plot(t,Delta_Theta(:,5)','r','LineWidth',1.5); grid on;
title("D \theta_5(t)"); ylabel("D \theta_5");
subplot(4,3,6);
plot(t,Delta_Theta(:,6)','r','LineWidth',1.5); grid on;
title("D \theta_6(t)"); ylabel("D \theta_6");

% Plot A
subplot(4,3,7);
plot(t(1:length(newT)),A_Theta(:,1)','r','LineWidth',1.5); grid on;
title("A \theta_1(t)");  ylabel("A \theta_1");
hold on
plot([1,length(A_Theta(:,1))],[Amax,Amax],'b--','LineWidth',1.5);
plot([1,length(A_Theta(:,1))],[-Amax,-Amax],'b--','LineWidth',1.5);
subplot(4,3,8);
plot(t(1:length(newT)),A_Theta(:,2)','r','LineWidth',1.5); grid on;
title("A \theta_2(t)");  ylabel("A \theta_2");
hold on
plot([1,length(A_Theta(:,1))],[Amax,Amax],'b--','LineWidth',1.5);
plot([1,length(A_Theta(:,1))],[-Amax,-Amax],'b--','LineWidth',1.5);
subplot(4,3,9);
plot(t(1:length(newT)),A_Theta(:,3)','r','LineWidth',1.5); grid on;
title("A \theta_3(t)");  ylabel("A \theta_3");
hold on
plot([1,length(A_Theta(:,1))],[Amax,Amax],'b--','LineWidth',1.5);
plot([1,length(A_Theta(:,1))],[-Amax,-Amax],'b--','LineWidth',1.5);
subplot(4,3,10);
plot(t(1:length(newT)),A_Theta(:,4)','r','LineWidth',1.5); grid on;
title("A \theta_4(t)");  ylabel("A \theta_4");
hold on
plot([1,length(A_Theta(:,1))],[Amax,Amax],'b--','LineWidth',1.5);
plot([1,length(A_Theta(:,1))],[-Amax,-Amax],'b--','LineWidth',1.5);
subplot(4,3,11);
plot(t(1:length(newT)),A_Theta(:,5)','r','LineWidth',1.5); grid on;
title("A \theta_5(t)");  ylabel("A \theta5");
hold on
plot([1,length(A_Theta(:,1))],[Amax,Amax],'b--','LineWidth',1.5);
plot([1,length(A_Theta(:,1))],[-Amax,-Amax],'b--','LineWidth',1.5);
hold on
plot([1,length(A_Theta(:,1))],[Amax,Amax],'b--','LineWidth',1.5);
plot([1,length(A_Theta(:,1))],[-Amax,-Amax],'b--','LineWidth',1.5);
subplot(4,3,12);
plot(t(1:length(newT)),A_Theta(:,6)','r','LineWidth',1.5); grid on;
title("A \theta_6(t)");  ylabel("A \theta_6");
hold on
plot([1,length(A_Theta(:,1))],[Amax,Amax],'b--','LineWidth',1.5);
plot([1,length(A_Theta(:,1))],[-Amax,-Amax],'b--','LineWidth',1.5);

% If adjuest
figure(2)
subplot(3,3,1)
hold on;
plot(time_line,alpha(1:length(newT)),'g--','LineWidth',1.5);
subplot(3,3,2)
hold on;
plot(time_line,beta(1:length(newT)),'g--','LineWidth',1.5);
subplot(3,3,3)
hold on;
plot(time_line,gamma(1:length(newT)),'g--','LineWidth',1.5);
subplot(3,3,4);
hold on
plot(time_line,Theta_Planned(1:length(newT),1),'y--','LineWidth',1.5);
subplot(3,3,5);
hold on
plot(time_line,Theta_Planned(1:length(newT),2),'y--','LineWidth',1.5);
subplot(3,3,6);
hold on
plot(time_line,Theta_Planned(1:length(newT),3),'y--','LineWidth',1.5);
subplot(3,3,7);
hold on
plot(time_line,Theta_Planned(1:length(newT),4),'y--','LineWidth',1.5);
subplot(3,3,8);
hold on
plot(time_line,Theta_Planned(1:length(newT),5),'y--','LineWidth',1.5);
subplot(3,3,9);
hold on
plot(time_line,Theta_Planned(1:length(newT),6),'y--','LineWidth',1.5);
figure(3)
subplot(4,3,1);
hold on;
plot(time_line,V_new(:,1)','g-','LineWidth',1); grid on;
subplot(4,3,2);
hold on;
plot(time_line,V_new(:,2)','g-','LineWidth',1); grid on;
subplot(4,3,3);
hold on;
plot(time_line,V_new(:,3)','g-','LineWidth',1); grid on;
subplot(4,3,4);
hold on;
plot(time_line,V_new(:,4)','g-','LineWidth',1); grid on;
subplot(4,3,5);
hold on;
plot(time_line,V_new(:,5)','g-','LineWidth',1); grid on;
subplot(4,3,6);
hold on;
plot(time_line,V_new(:,6)','g-','LineWidth',1); grid on;
subplot(4,3,7);
hold on;
plot(time_line,A_new(:,1)','g-','LineWidth',1); grid on;
subplot(4,3,8);
hold on;
plot(time_line,A_new(:,2)','g-','LineWidth',1); grid on;
subplot(4,3,9);
hold on;
plot(time_line,A_new(:,3)','g-','LineWidth',1); grid on;
subplot(4,3,10);
hold on;
plot(time_line,A_new(:,4)','g-','LineWidth',1); grid on;
subplot(4,3,11);
hold on;
plot(time_line,A_new(:,5)','g-','LineWidth',1); grid on;
subplot(4,3,12);
hold on;
plot(time_line,A_new(:,6)','g-','LineWidth',1); grid on;
%% Show
pause
Theta_Planned=[Theta_Planned,zeros([length(Theta_Planned(:,1)),1])];
Theta_Planned=Theta_Planned*pi/180;

figure(1)
plot3(current(4),current(5),current(6),'*','LineWidth',1.5);
hold on;
robot.plot(Theta_Planned,'jointdiam',1,'arrow','jaxes','wrist');

% data=Theta_Planned;
% for idx = 1:1:length(data(:,1))
%     robot.plot(data(idx,:),'jointdiam',1,'arrow','jaxes','wrist');
%     drawnow;
%     im(idx) = getframe(1);
% end
% v = VideoWriter('Final_long.avi', 'Motion JPEG AVI');
% open(v);
% writeVideo(v, im);
% close(v);
% clear v;
