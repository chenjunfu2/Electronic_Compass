[ax, ay, az, gx, gy, gz, mx, my, mz]=textread("convert2.txt","%d %d %d %d %d %d %d %d %d");

%保存原始raw数据
rx=mx;
ry=my;
rz=mz;

%灵敏度校准（数值由厂家写入芯片，固定值）

adjx=181;
adjy=181;
adjz=170;

mx=mx*((((adjx-128)*0.5)/128)+1);
my=my*((((adjy-128)*0.5)/128)+1);
mz=mz*((((adjz-128)*0.5)/128)+1);

%绘图准备
clf;
hold on;
grid on;
axis equal;
xlabel('X轴');
ylabel('Y轴');
zlabel('Z轴');
%plot3(ax, ay, az, '.r');
%plot3(gx, gy, gz, '.g');

%plot3(rx, ry, rz, '.r');
%plot3(mx, my, mz, '.b');

x=mx;
y=my;
z=mz;

[VOX, VOY, VOZ, VRX, VRY, VRZ] = ellipsoidFit(x,y,z);%求解椭球参数

%拟合结果的可视化显示
%ellipsoid(VOX, VOY, VOZ, VRX, VRY, VRZ, 50);% ellipsoid(OX, OY, OZ, ra, rb, rc, 50);
%alpha(0.01)
%plot3([VOX - VRX, VOX + VRX], [VOY, VOY], [VOZ, VOZ], 'LineWidth',5);
%plot3([VOX, VOX], [VOY - VRY, VOY + VRY], [VOZ, VOZ],  'LineWidth',5);
%plot3([VOX, VOX], [VOY, VOY], [VOZ - VRZ, VOZ + VRZ],  'LineWidth', 5);

fprintf('拟合结果: XYZ中心[%0.2f, %0.2f, %0.2f], 半轴长[%1.2f, %1.2f, %1.2f]\n', VOX, VOY, VOZ, VRX, VRY, VRZ);

%从fig文件中读取数据
%g=open('untitled.fig');
%c=get(g,"Children");
%l=get(c,"Children");
%
%m2x=get(l,"XData");
%m2y=get(l,"YData");
%m2z=get(l,"ZData");
%
%m2x=cell2mat(m2x);
%m2y=cell2mat(m2y);
%m2z=cell2mat(m2z);

m2x=mx;
m2y=my;
m2z=mz;

%使用极值和除二校准
offx=(max(mx)+min(mx))/2;
offy=(max(my)+min(my))/2;
offz=(max(mz)+min(mz))/2;

ox=m2x-offx;
oy=m2y-offy;
oz=m2z-offz;


%使用椭球校准
vk=(VRX+VRY+VRZ)/3;%使用3轴平均数作为倍率

vx=(m2x-VOX)/VRX*vk;
vy=(m2y-VOY)/VRY*vk;
vz=(m2z-VOZ)/VRZ*vk;

%绘图准备
%clf;
%hold on;
%grid on;
%axis equal;
%xlabel('X轴');
%ylabel('Y轴');
%zlabel('Z轴');

%校准结果可视化
%plot3(m2x, m2y, m2z, '.r');
plot3(ox, oy, oz, '.g');
plot3(vx, vy, vz, '.b');

%plot3(vx, vy, vz, '.b');
%plot3(mx, my, mz, '.r');

[VOX, VOY, VOZ, VRX, VRY, VRZ] = ellipsoidFit(vx,vy,vz);%求解椭球参数

%拟合结果的可视化显示
ellipsoid(VOX, VOY, VOZ, VRX, VRY, VRZ, 50);% ellipsoid(OX, OY, OZ, ra, rb, rc, 50);
alpha(0.01)
plot3([VOX - VRX, VOX + VRX], [VOY, VOY], [VOZ, VOZ], 'LineWidth',5);
plot3([VOX, VOX], [VOY - VRY, VOY + VRY], [VOZ, VOZ],  'LineWidth',5);
plot3([VOX, VOX], [VOY, VOY], [VOZ - VRZ, VOZ + VRZ],  'LineWidth', 5);

fprintf('椭球校准结果: XYZ中心[%0.2f, %0.2f, %0.2f], 半轴长[%1.2f, %1.2f, %1.2f]\n', VOX, VOY, VOZ, VRX, VRY, VRZ);

[VOX, VOY, VOZ, VRX, VRY, VRZ] = ellipsoidFit(ox,oy,oz);%求解椭球参数

%拟合结果的可视化显示
ellipsoid(VOX, VOY, VOZ, VRX, VRY, VRZ, 50);% ellipsoid(OX, OY, OZ, ra, rb, rc, 50);
alpha(0.01)
plot3([VOX - VRX, VOX + VRX], [VOY, VOY], [VOZ, VOZ], 'LineWidth',5);
plot3([VOX, VOX], [VOY - VRY, VOY + VRY], [VOZ, VOZ],  'LineWidth',5);
plot3([VOX, VOX], [VOY, VOY], [VOZ - VRZ, VOZ + VRZ],  'LineWidth', 5);

fprintf('off校准结果: XYZ中心[%0.2f, %0.2f, %0.2f], 半轴长[%1.2f, %1.2f, %1.2f]\n', VOX, VOY, VOZ, VRX, VRY, VRZ);