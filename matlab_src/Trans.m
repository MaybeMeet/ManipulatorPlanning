function [T] = Trans(alpha,a,d,theta)
%UNTITLED2 �������(��ֵ��������ű���)
%  DH������: alpha, a, d, theta
%  ���� ��������ϵ��������α任
T=[cosd(theta),-sind(theta),0,a;
    sind(theta)*cosd(alpha), cosd(theta)*cosd(alpha),-sind(alpha),-sind(alpha)*d;
    sind(theta)*sind(alpha),cosd(theta)*sind(alpha),cosd(alpha),cosd(alpha)*d;
    0,0,0,1];
end

