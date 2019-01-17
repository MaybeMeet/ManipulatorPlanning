function [alpha,beta,gamma] = R2Eular(R)
% Cvt R to ZYZ Eular angle
p=rotm2eul(R,'ZYZ')*180/pi;
alpha=p(1);
beta=p(2);
gamma=p(3);
end

