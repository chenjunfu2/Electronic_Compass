function [VOX, VOY, VOZ, VRX, VRY, VRZ] = ellipsoidFit(x,y,z)
%�������
%�βΣ�xyzΪҪ��ϵ���ά�������ݣ���������������, sampleΪNx3
%����ֵ��Ox, Oy, OzΪ��ϳ����������ģ�Rx, Ry, RzΪ��������ĳ���

% x = sample(:, 1);
% y = sample(:, 2);
% z = sample(:, 3);

K = [y.^2, z.^2, x, y, z, ones(length(x), 1)];
Y = -x.^2;
V=(K'*K)^(-1)*K'*Y;

VOX = -V(3)/2;% -C/2
VOY = -V(4)/V(1)/2;% -D/A/2
VOZ= -V(5)/V(2)/2;% -E/B/2
VRX = sqrt(VOX^2 + V(1)*VOY^2 + V(2)*VOZ^2 - V(6));% sqrt(VOX^2 + A*VOY^2 + B*VOZ^2 - F)
VRY = sqrt(VRX^2/V(1));% sqrt(VRX^2/A);
VRZ = sqrt(VRX^2/V(2));% sqrt(VRX^2/B);
end
