%%  Explore the space
clear
Data=[];% [a,b,g,x,y,z,t1-t6, true t1-t6];
load lim.mat
sl=5;
t1=lim1(1):(lim1(2)-lim1(1))/sl:lim1(2);
t2=lim2(1):(lim2(2)-lim2(1))/sl:lim2(2);
t3=lim3(1):(lim3(2)-lim3(1))/sl:lim3(2);
t4=lim4(1):(lim4(2)-lim4(1))/sl:lim4(2);
t5=lim5(1):(lim5(2)-lim5(1))/sl:lim5(2);
t6=lim6(1):(lim6(2)-lim6(1))/sl:lim6(2);
total=length(t1)*length(t2)*length(t3)*length(t4)*length(t5)*length(t6);
fprintf("Total: %d\n",total);
count=0;
for tt1=lim1(1):(lim1(2)-lim1(1))/sl:lim1(2)
    for tt2=lim2(1):(lim2(2)-lim2(1))/sl:lim2(2)
        for tt3=lim3(1):(lim3(2)-lim3(1))/sl:lim3(2)
            for tt4=lim4(1):(lim4(2)-lim4(1))/sl:lim4(2)
                for tt5=lim5(1):(lim5(2)-lim5(1))/sl:lim5(2)
                    for tt6=lim6(1):(lim6(2)-lim6(1))/sl:lim6(2)
                        fprintf("-----------------------------------------------------\n");
                        fprintf("||%d/%d||",count,total);
                        fprintf("Theta :|%d|%d|%d|%d|%d|%d|\n",tt1,tt2,tt3,tt4,tt5,tt6);
                        pose=Joint2Pose(tt1,tt2,tt3,tt4,tt5,tt6);
                        param=InvK(pose(1),pose(2),pose(3),pose(4),pose(5),pose(6));
                        fprintf("thetaX:");
                        disp(param);
                        theta=[tt1,tt2,tt3,tt4,tt5,tt6];
                        pose=ones([length(param(:,1)),1])*pose;
                        theta=ones([length(param(:,1)),1])*theta;
                        Data=[Data;pose,param,theta];
                        count = count+1;
                        if mod(count,100)==0
                            %fprintf("Finish %d/%d \n",count,total);
                            %disp([pose,param]);
                            save Data.mat  Data;
                        end
                    end
                end
            end
        end
    end
end
