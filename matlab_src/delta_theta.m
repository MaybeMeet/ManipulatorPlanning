function [D] = delta_theta(theta1,theta2)
% 度量非线性的旋转角度的距离
d=[];
for idx = 1:length(theta1)
    d(idx)=min([abs(theta1(idx)-theta2(idx)),abs(theta1(idx)-360-theta2(idx))...
        ,abs(theta1(idx)+360-theta2(idx))]);
end
D=sum(d.^2);
end