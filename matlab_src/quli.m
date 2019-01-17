function [xt] =quli(t,rtb,xi,xf)
%  �������Բ岹
%  t��ʱ����Ƭ, ĩ��ʱ�����tf , ���ص�xt��ʱ������, rtb��tbռ��ʱ��tf�ı���
%  xi xf �ǳ�ʼĩ��Ŀ��ֵ
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

