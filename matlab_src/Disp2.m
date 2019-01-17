%% Plot the space exploration
clear
load washed_space.mat data
load lim.mat
L1=RevoluteMDH('alpha',0,    'a',0,'d',0,'qlim',lim1);
L2=RevoluteMDH('alpha',-pi/2,'a',0,'d',0,'qlim',lim2);
L3=RevoluteMDH('alpha',0,    'a',8,'d',2,'qlim',lim3);
L4=RevoluteMDH('alpha',-pi/2,'a',0,'d',8,'qlim',lim4);
L5=RevoluteMDH('alpha',pi/2  ,'a',0,'d',0,'qlim',lim5);
L6=RevoluteMDH('alpha',pi/2,  'a',0,'d',0,'qlim',lim6);
L7=RevoluteMDH('alpha',0,     'a',0,'d',2,'qlim',[0,0]);
robot=SerialLink([L1,L2,L3,L4,L5,L6,L7]);
data=[data,zeros([length(data(:,1)),1])];
figure(1)
title("Space exploration and InvK demo")
count=1;

for idx = 1:10:length(data(:,1))
    robot.plot(data(idx,:),'jointdiam',1,'arrow','jaxes','wrist');
    drawnow;
    im(count) = getframe(1);
    count=count+1;
    idx
    if mod(count,500)==0
        v = VideoWriter('demo.avi', 'Motion JPEG AVI'); % 设定名称、格式
        open(v);
        writeVideo(v, im);
        close(v);
        clear v;
    end
end
