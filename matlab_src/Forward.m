%% Initialize
clear;
syms t1 t2 t3 t4 t5 t6 % Joint Var

%% Get Trans MTX
joint_number = 7;
% Get adjecent Transfer mtx
T01=Trans(0,0,0,t1);
T12=Trans(-90,0,0,t2);
T23=Trans(-0,8,2,t3);
T34=Trans(-90,0,8,t4);
T45=Trans(90,0,0,t5);
T56=Trans(90,0,0,t6);
T67=Trans(0,0,2,0);
T_adj={T01,T12,T23,T34,T45,T56,T67};
%% Forward
T=[];
temp=eye(4);
for i = 1:joint_number
    temp = temp*T_adj{i};
    T{i}=temp;
end
clear temp;
save T.mat T