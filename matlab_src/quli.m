function [xt] =quli(t,rtb,xi,xf)
%  二次线性插补
%  t是时间切片, 末端时间就是tf , 返回的xt是时间序列, rtb是tb占总时长tf的比例
%  xi xf 是初始末端目标值
tf=t(end);
tb=tf*rtb;
a=(xf-xi)/(tb*(tf-tb));
T1=t(1:floor(tb/(t(2)-t(1))));
T2=t(floor(tb/(t(2)-t(1)))+1:ceil((tf-tb)/(t(2)-t(1))));
T3=t(ceil((tf-tb)/(t(2)-t(1))):end);
x1=xi+0.5*a.*T1.^2;
x2=xi+a*tb.*T2-0.5*a*tb^2;
x3=xf-0.5*a.*(tf-T3).^2;
xt=[x1,x2,x3(2:end)];
xt=xt(1:length(t));
end

