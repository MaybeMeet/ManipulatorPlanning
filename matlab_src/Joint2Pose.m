function [pose] = Joint2Pose(tt1,tt2,tt3,tt4,tt5,tt6)
%  用前向传播从关节变量得到Pose (ZY'X')欧拉角表示
% syms t1 t2 t3 t4 t5 t6
% T_val=double(subs(T,[t1,t2,t3,t4,t5,t6],[tt1,tt2,tt3,tt4,tt5,tt6]));
% R=T_val(1:3,1:3);
% [a,b,g]=R2Eular(R);
% pose=[a,b,g,T_val(1:3,4)'];
t1=tt1;t2=tt2;t3=tt3;t4=tt4;t5=tt5;t6=tt6;
r1=[ - sin((pi*t6)/180)*(sin((pi*t4)/180)*(cos((pi*t1)/180)*sin((pi*t2)/180)*sin((pi*t3)/180) - cos((pi*t1)/180)*cos((pi*t2)/180)*cos((pi*t3)/180)) + cos((pi*t4)/180)*sin((pi*t1)/180)) - cos((pi*t6)/180)*(cos((pi*t5)/180)*(cos((pi*t4)/180)*(cos((pi*t1)/180)*sin((pi*t2)/180)*sin((pi*t3)/180) - cos((pi*t1)/180)*cos((pi*t2)/180)*cos((pi*t3)/180)) - sin((pi*t1)/180)*sin((pi*t4)/180)) + sin((pi*t5)/180)*(cos((pi*t1)/180)*cos((pi*t2)/180)*sin((pi*t3)/180) + cos((pi*t1)/180)*cos((pi*t3)/180)*sin((pi*t2)/180))),...
    sin((pi*t6)/180)*(cos((pi*t5)/180)*(cos((pi*t4)/180)*(cos((pi*t1)/180)*sin((pi*t2)/180)*sin((pi*t3)/180) - cos((pi*t1)/180)*cos((pi*t2)/180)*cos((pi*t3)/180)) - sin((pi*t1)/180)*sin((pi*t4)/180)) + sin((pi*t5)/180)*(cos((pi*t1)/180)*cos((pi*t2)/180)*sin((pi*t3)/180) + cos((pi*t1)/180)*cos((pi*t3)/180)*sin((pi*t2)/180))) - cos((pi*t6)/180)*(sin((pi*t4)/180)*(cos((pi*t1)/180)*sin((pi*t2)/180)*sin((pi*t3)/180) - cos((pi*t1)/180)*cos((pi*t2)/180)*cos((pi*t3)/180)) + cos((pi*t4)/180)*sin((pi*t1)/180)),...
    cos((pi*t5)/180)*(cos((pi*t1)/180)*cos((pi*t2)/180)*sin((pi*t3)/180) + cos((pi*t1)/180)*cos((pi*t3)/180)*sin((pi*t2)/180)) - sin((pi*t5)/180)*(cos((pi*t4)/180)*(cos((pi*t1)/180)*sin((pi*t2)/180)*sin((pi*t3)/180) - cos((pi*t1)/180)*cos((pi*t2)/180)*cos((pi*t3)/180)) - sin((pi*t1)/180)*sin((pi*t4)/180)),...
    2*cos((pi*t5)/180)*(cos((pi*t1)/180)*cos((pi*t2)/180)*sin((pi*t3)/180) + cos((pi*t1)/180)*cos((pi*t3)/180)*sin((pi*t2)/180)) - 2*sin((pi*t5)/180)*(cos((pi*t4)/180)*(cos((pi*t1)/180)*sin((pi*t2)/180)*sin((pi*t3)/180) - cos((pi*t1)/180)*cos((pi*t2)/180)*cos((pi*t3)/180)) - sin((pi*t1)/180)*sin((pi*t4)/180)) - 2*sin((pi*t1)/180) + 8*cos((pi*t1)/180)*cos((pi*t2)/180) - 8*cos((pi*t1)/180)*cos((pi*t2)/180)*sin((pi*t3)/180) - 8*cos((pi*t1)/180)*cos((pi*t3)/180)*sin((pi*t2)/180)];
r2=[   sin((pi*t6)/180)*(cos((pi*t1)/180)*cos((pi*t4)/180) + sin((pi*t4)/180)*(cos((pi*t2)/180)*cos((pi*t3)/180)*sin((pi*t1)/180) - sin((pi*t1)/180)*sin((pi*t2)/180)*sin((pi*t3)/180))) + cos((pi*t6)/180)*(cos((pi*t5)/180)*(cos((pi*t4)/180)*(cos((pi*t2)/180)*cos((pi*t3)/180)*sin((pi*t1)/180) - sin((pi*t1)/180)*sin((pi*t2)/180)*sin((pi*t3)/180)) - cos((pi*t1)/180)*sin((pi*t4)/180)) - sin((pi*t5)/180)*(cos((pi*t2)/180)*sin((pi*t1)/180)*sin((pi*t3)/180) + cos((pi*t3)/180)*sin((pi*t1)/180)*sin((pi*t2)/180))),...
    cos((pi*t6)/180)*(cos((pi*t1)/180)*cos((pi*t4)/180) + sin((pi*t4)/180)*(cos((pi*t2)/180)*cos((pi*t3)/180)*sin((pi*t1)/180) - sin((pi*t1)/180)*sin((pi*t2)/180)*sin((pi*t3)/180))) - sin((pi*t6)/180)*(cos((pi*t5)/180)*(cos((pi*t4)/180)*(cos((pi*t2)/180)*cos((pi*t3)/180)*sin((pi*t1)/180) - sin((pi*t1)/180)*sin((pi*t2)/180)*sin((pi*t3)/180)) - cos((pi*t1)/180)*sin((pi*t4)/180)) - sin((pi*t5)/180)*(cos((pi*t2)/180)*sin((pi*t1)/180)*sin((pi*t3)/180) + cos((pi*t3)/180)*sin((pi*t1)/180)*sin((pi*t2)/180))),...
    sin((pi*t5)/180)*(cos((pi*t4)/180)*(cos((pi*t2)/180)*cos((pi*t3)/180)*sin((pi*t1)/180) - sin((pi*t1)/180)*sin((pi*t2)/180)*sin((pi*t3)/180)) - cos((pi*t1)/180)*sin((pi*t4)/180)) + cos((pi*t5)/180)*(cos((pi*t2)/180)*sin((pi*t1)/180)*sin((pi*t3)/180) + cos((pi*t3)/180)*sin((pi*t1)/180)*sin((pi*t2)/180)),...
    2*cos((pi*t1)/180) + 2*sin((pi*t5)/180)*(cos((pi*t4)/180)*(cos((pi*t2)/180)*cos((pi*t3)/180)*sin((pi*t1)/180) - sin((pi*t1)/180)*sin((pi*t2)/180)*sin((pi*t3)/180)) - cos((pi*t1)/180)*sin((pi*t4)/180)) + 2*cos((pi*t5)/180)*(cos((pi*t2)/180)*sin((pi*t1)/180)*sin((pi*t3)/180) + cos((pi*t3)/180)*sin((pi*t1)/180)*sin((pi*t2)/180)) + 8*cos((pi*t2)/180)*sin((pi*t1)/180) - 8*cos((pi*t2)/180)*sin((pi*t1)/180)*sin((pi*t3)/180) - 8*cos((pi*t3)/180)*sin((pi*t1)/180)*sin((pi*t2)/180)];
r3=[ - cos((pi*t6)/180)*(sin((pi*t5)/180)*(cos((pi*t2)/180)*cos((pi*t3)/180) - sin((pi*t2)/180)*sin((pi*t3)/180)) + cos((pi*t4)/180)*cos((pi*t5)/180)*(cos((pi*t2)/180)*sin((pi*t3)/180) + cos((pi*t3)/180)*sin((pi*t2)/180))) - sin((pi*t4)/180)*sin((pi*t6)/180)*(cos((pi*t2)/180)*sin((pi*t3)/180) + cos((pi*t3)/180)*sin((pi*t2)/180)),...
    sin((pi*t6)/180)*(sin((pi*t5)/180)*(cos((pi*t2)/180)*cos((pi*t3)/180) - sin((pi*t2)/180)*sin((pi*t3)/180)) + cos((pi*t4)/180)*cos((pi*t5)/180)*(cos((pi*t2)/180)*sin((pi*t3)/180) + cos((pi*t3)/180)*sin((pi*t2)/180))) - cos((pi*t6)/180)*sin((pi*t4)/180)*(cos((pi*t2)/180)*sin((pi*t3)/180) + cos((pi*t3)/180)*sin((pi*t2)/180)),...
    cos((pi*t5)/180)*(cos((pi*t2)/180)*cos((pi*t3)/180) - sin((pi*t2)/180)*sin((pi*t3)/180)) - cos((pi*t4)/180)*sin((pi*t5)/180)*(cos((pi*t2)/180)*sin((pi*t3)/180) + cos((pi*t3)/180)*sin((pi*t2)/180)), ...
    2*cos((pi*t5)/180)*(cos((pi*t2)/180)*cos((pi*t3)/180) - sin((pi*t2)/180)*sin((pi*t3)/180)) - 8*sin((pi*t2)/180) - 8*cos((pi*t2)/180)*cos((pi*t3)/180) + 8*sin((pi*t2)/180)*sin((pi*t3)/180) - 2*cos((pi*t4)/180)*sin((pi*t5)/180)*(cos((pi*t2)/180)*sin((pi*t3)/180) + cos((pi*t3)/180)*sin((pi*t2)/180))];
R=[r1(1:3);r2(1:3);r3(1:3)];
[a,b,g]=R2Eular(R);
pose=[a,b,g,r1(end),r2(end),r3(end)];
end

