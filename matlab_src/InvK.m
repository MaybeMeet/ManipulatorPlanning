function [P] = InvK(alpha,beta,gamma,x,y,z)
% 逆运动学求解, 输入末端ZY'X'欧拉角
    %% Init and Sovle {4} Porg
    %load lim.mat ;
    lim1=[-90,90];
    lim2=[-45,45];
    %lim3=[-135,-45];
    lim3=[-170,-10];
    lim4=[-180,180];
    lim5=[0,180];
    lim6=[-180,180];
    param=[];
    pose_true=[alpha,beta,gamma,x,y,z];
    % Get {4}  Porg
    R06=Eular2R(alpha,beta,gamma);
    P4org=[x,y,z]'-R06*[0,0,2]';
   %% Solve Arm
    % Solve theta3 <=4 solutions
    r2=P4org'*P4org;
%     u=roots([2*64-(r2-4),-4*64,4*64-2*(r2-4),-4*64,2*64-(r2-4)])
    t3a=132-r2;t3b=-256; t3c=t3a;
    u=[1/(2*t3a)*(-t3b+sqrt(t3b^2-4*t3a*t3c)),1/(2*t3a)*(-t3b-sqrt(t3b^2-4*t3a*t3c))];   
    theta3=[];
    for idx = 1:2
        tt3=u(idx);
        if abs(imag(tt3))<1e-4
            tt3=real(tt3);
            t3=2*atand(tt3);
            if t3>lim3(1)-0.5 && t3<lim3(2)+0.5
                theta3=[theta3,real(t3)];
            end
        end
    end
    theta3=unique(theta3);
    if isempty(theta3)
        P=[NaN,NaN,NaN,NaN,NaN,NaN];
        return;
    end
    % Solve theta 2
    theta23=[];
    for idx = 1:1:length(theta3)
        % For each theta3
        t3=theta3(idx);
        f1=-8*sind(t3)+8;
        f2=8*cosd(t3);
        f3=2;
        % z=-s_2f_1-c_2f_2
        % z*(1+u^2)=-2*u*f1-(1-u^2)*f2
        % (f2-z)u^2-2*f1*u-(f2+z)=0
        %t2=double(solve(-sind(X)*f1-cosd(X)*f2==P4org(3)));
        %tt2=roots([f2-P4org(3),-2*f1,-(f2+P4org(3))]);
        t2a=f2-P4org(3);t2b=-2*f1; t2c=-(f2+P4org(3));
        tt2=[1/(2*t2a)*(-t2b+sqrt(t2b^2-4*t2a*t2c)),1/(2*t2a)*(-t2b-sqrt(t2b^2-4*t2a*t2c))];
        tt2=real(tt2);
        t2=atand(tt2);
        t2 = t2*2;
        % check t2
        for jdx=1:1:length(t2)
            if t2(jdx)>lim2(1)-0.5 && t2(jdx)<lim2(2)+0.5
                theta23=[theta23;real(t2(jdx)),t3];
            end
        end
    end
    if isempty(theta23)
        P=[NaN,NaN,NaN,NaN,NaN,NaN];
        return;
    end
    % Solve theta1
    theta123=[];
    for idx = 1:1:length(theta23(:,1))
        % For each t2,t3 combination
        t3=theta23(idx,2);
        t2=theta23(idx,1);
        f1=-8*sind(t3)+8;
        f2=8*cosd(t3);
        f3=2;
        g1=cosd(t2)*f1-sind(t2)*f2;
        g2=f3;
        g3=-sind(t2)*f1-cosd(t2)*f2;
        % x=c1g1-s1g2
        % (1+u^2)*x=(1-u^2)*g1-2*u*g2
        % (x+g1)*u^2 + 2*g2*u + (x-g1)
        % t1=double(solve(cosd(X)*g1-sind(X)*g2==P4org(1)));
        tt1=roots([P4org(1)+g1,2*g2,P4org(1)-g1]);
        t1a=P4org(1)+g1;t1b=2*g2;t1c=P4org(1)-g1;
        tt1=[1/(2*t1a)*(-t1b+sqrt(t1b^2-4*t1a*t1c)),1/(2*t1a)*(-t1b-sqrt(t1b^2-4*t1a*t1c))];
        tt1=real(tt1);
        t1=atand(tt1);
        t1=t1*2;
        for jdx=1:1:length(t1)
            if t1(jdx)>lim1(1)-0.5 && t1(jdx)<lim1(2)+0.5
                theta123=[theta123;real(t1(jdx)),t2,t3];
            end
        end
    end
    if isempty(theta123)
        P=[NaN,NaN,NaN,NaN,NaN,NaN];
        return;
    end
    %%  Solve hand
    clear t1 t2 t3 t4;
    syms t1 t2 t3 t4;
    for idx = 1:1:length(theta123(:,1))
        % Get R04|t4=0
        %T_valed=double(subs(T{4},[t1,t2,t3,t4],[theta123(idx,1),theta123(idx,2),theta123(idx,3),0]));
        T_valued=T4forward(theta123(idx,1),theta123(idx,2),theta123(idx,3),0);
        R_valued=T_valued(1:3,1:3);
        R46=R_valued'*R06;
        [aa,bb,gg]=R2Eular(R46);
        tt4=180+aa;
        tt5=180+bb;
        tt6=gg;
        % Check 
       if tt5>lim5(1)-0.5 && tt5<lim5(2)+0.5
           param=[param;[theta123(idx,:),tt4,tt5,tt6]]; 
       end
    end
    
    %% Choose correct 
    P=[];
    if isempty(param)
        P=[NaN,NaN,NaN,NaN,NaN,NaN];
        return;
    end
    for idx = 1:1:length(param(:,1))
        pose = Joint2Pose(param(idx,1),param(idx,2),param(idx,3),param(idx,4),param(idx,5),param(idx,6));
        error=sqrt((pose-pose_true)*(pose-pose_true)');
        if error<1e-4
            P=[P;param(idx,:)];
        end
    end
    if size(P)==0
        P=[NaN,NaN,NaN,NaN,NaN,NaN];
    end
return;
end

