function [R] = Eular2R(alpha,beta,gamma)
% ŷ���ǵõ���ת����
R=eul2rotm([alpha*pi/180,beta*pi/180,gamma*pi/180],'ZYZ');
end

