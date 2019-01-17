function [R] = Eular2R(alpha,beta,gamma)
% 欧拉角得到旋转矩阵
R=eul2rotm([alpha*pi/180,beta*pi/180,gamma*pi/180],'ZYZ');
end

