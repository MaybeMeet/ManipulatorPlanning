function [d] =VTheta(theta1,theta2)
% ����6���ؽڱ����ı仯�� theta1-theta2
    d=[];
    for idx = 1:length(theta1)
        d(idx)=min([abs(theta1(idx)-theta2(idx)),abs(theta1(idx)-360-theta2(idx))...
            ,abs(theta1(idx)+360-theta2(idx))]);
    end
end

