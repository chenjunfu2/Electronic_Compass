% %����ģ������
OX = 100;
OY = 123;
OZ = 345;
Rx = 100;
Ry = 200;
Rz = 150;
fprintf('��ʵֵ  : XYZ����[%0.2f, %0.2f, %0.2f], ���᳤[%1.2f, %1.2f, %1.2f]\n', OX, OY, OZ, Rx, Ry, Rz);
x = [];
y = [];
z = [];
for theta = 0: 0.1: 2*pi%����
    for phi = 0: 0.1: pi%γ��
        x = [x, Rx * cos(theta) * sin(phi) + OX];
        y = [y, Ry * sin(theta) * sin(phi) + OY];
        z = [z, Rz * cos(phi) + OZ];
    end
end

% %unifrnd(a,b,m,n)%����m��n��[a,b]֮��������
error = 10;
x = x + unifrnd(-error, error, 1, length(x));
y = y + unifrnd(-error, error, 1, length(y));
z = z + unifrnd(-error, error, 1, length(z));
x = x';
y = y';
z = z';
%���ˣ�ģ�����ά��������������ϡ�ʵ��Ӧ��ʱ����Щ��ά������Ӧ�ɴ�����������á�

clf;
set(figure(1),'NumberTitle','off','Name','�������') ;
%��ʾģ������
clf;
hold on;
grid on;
axis equal;
xlabel('X��');
ylabel('Y��');
zlabel('Z��');
plot3(x,y,z, '.b');
%plot3(x,y,z, 'or');
%plot3(x,y,z, '*r');
[VOX, VOY, VOZ, VRX, VRY, VRZ] = ellipsoidFit(x,y,z);%����������

%��Ͻ���Ŀ��ӻ���ʾ
ellipsoid(VOX, VOY, VOZ, VRX, VRY, VRZ, 50);% ellipsoid(OX, OY, OZ, ra, rb, rc, 50);
alpha(0.01)
plot3([VOX - VRX, VOX + VRX], [VOY, VOY], [VOZ, VOZ], 'LineWidth',5);
plot3([VOX, VOX], [VOY - VRY, VOY + VRY], [VOZ, VOZ],  'LineWidth',5);
plot3([VOX, VOX], [VOY, VOY], [VOZ - VRZ, VOZ + VRZ],  'LineWidth', 5);

fprintf('��Ͻ��: XYZ����[%0.2f, %0.2f, %0.2f], ���᳤[%1.2f, %1.2f, %1.2f]\n', VOX, VOY, VOZ, VRX, VRY, VRZ);

%����ϵĽ��������������
Rsphere = sqrt(((x - VOX)/VRX).^2 + ((y - VOY)/VRY).^2 +((z - VOZ)/VRZ).^2);%ÿһ��������Ĺ�һ����
figure(3);
%Rsphere(find(Rsphere>1.1)) = [];%�������������ɾ��
hist(Rsphere,50)
Rerror = abs(Rsphere - 1);

fprintf('������: %1.2f%%, ƽ�����: %1.2f%% ,����׼��: %1.7f\n', max(Rerror) * 100, mean(Rerror) * 100, std(Rerror));





