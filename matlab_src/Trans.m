function [T] = Trans(alpha,a,d,theta)
%UNTITLED2 输入参数(数值变量或符号变量)
%  DH参数表: alpha, a, d, theta
%  返回 相邻坐标系的正向其次变换
T=[cosd(theta),-sind(theta),0,a;
    sind(theta)*cosd(alpha), cosd(theta)*cosd(alpha),-sind(alpha),-sind(alpha)*d;
    sind(theta)*sind(alpha),cosd(theta)*sind(alpha),cosd(alpha),cosd(alpha)*d;
    0,0,0,1];
end

