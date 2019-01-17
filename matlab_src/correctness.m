%% Correctness
clear;
clc
load T.mat
Theta=[...
    20,10,-135,10,80,10;...
    0,0,-45,0,90,0;...
    0,0,-90,0,90,0;...
    0,0,-125,0,90,0;...
    10,10,-125,10,45,0;...
    30,40,-90,10,45,0;...
    ];
ThetaX=[];
for idx = 1:1:length(Theta(:,1))
    theta=Theta(idx,:);
    pose=Joint2Pose(theta(1),theta(2),theta(3),theta(4),theta(5),theta(6));
    syms t1 t2 t3 t4;
    thetaX=InvK(pose(1),pose(2),pose(3),pose(4),pose(5),pose(6));
    ThetaX=[ThetaX;thetaX];
    posex=[];
    for jdx = 1:1:length(thetaX(:,1))
        posex=[posex;Joint2Pose(thetaX(jdx,1),thetaX(jdx,2),thetaX(jdx,3),thetaX(jdx,4),thetaX(jdx,5),thetaX(jdx,6))];
    end
    fprintf("-------------------------------------------------------------------\n");
    %fprintf("-------|Pose:\n");
    %disp(pose);
    %fprintf("-------|PoseX\n");
    %disp(posex);
    fprintf("-------|Theta\n");
    disp(theta);
    fprintf("-------|ThetaX\n");
    disp(thetaX);
    %fprintf("-------|Delta:\n");
    %disp(thetaX-ones([length(thetaX(:,1)),1])*theta);
end