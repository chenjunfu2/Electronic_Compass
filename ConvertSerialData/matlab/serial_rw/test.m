s = serialport("COM9",115200,"Timeout",10);
s.flush();

write(s,'S',"char");%写入开始符号

readline(s);%读取并抛掉第一行的初始化完毕信息

adj=read(s,3,"uint8");%读取3个校准值
fprintf("adj:%d,%d,%d",adj(1),adj(2),adj(3));



%file=fopen('write.bin','wb');
%fwrite(file,adj,"uint8");%写入文件
%
%while true
%    data=read(s,9,"int16");
%    fwrite(file,data,"int16");
%end



%绘图准备
clf;
%hold on;
%grid on;
%axis equal;
%xlabel('X轴');
%ylabel('Y轴');
%zlabel('Z轴');

%使用3轴数据作为空间坐标

while true
    data0=read(s,3,"int16");%加速度计
    data1=read(s,3,"int16");%陀螺仪
    data2=read(s,3,"int16");%磁力计
    
    ax=data0(1);
    ay=data0(2);
    az=data0(3);

    %水平计算
    roll=atan2(ay,az);
    pitch=atan2(ax,az);

    %roll=atan2(ay,sqrt(ax*ax+az*az));
    %pitch=atan2(ax,sqrt(ay*ay+az*az));

    fprintf("roll=%f,pitch=%f,",roll*180/pi,pitch*180/pi);


    mx=data2(1);
    my=data2(2);
    mz=data2(3);

    %厂家灵敏度校准
    mx=mx*((((adj(1)-128)*0.5)/128)+1);
    my=my*((((adj(2)-128)*0.5)/128)+1);
    mz=mz*((((adj(3)-128)*0.5)/128)+1);


    %校准值
    VOX=101.09;
    VOY=-2.43;
    VOZ=234.65;

    VRX=269.56;
    VRY=283.29;
    VRZ=242.96;

    %vk=(VRX+VRY+VRZ)/3;
    vk=1;

    %椭球校准
    mx=(mx-VOX)/VRX*vk;
    my=(my-VOY)/VRY*vk;
    mz=(mz-VOZ)/VRZ*vk;

    %plot3(mx,my,mz, '.b');%绘制
    fprintf("mx=%f,my=%f,mz=%f,",mx,my,mz);

    %倾斜校准
    hy=mx*cos(pitch)+my*sin(roll)*sin(pitch)-mz*cos(roll)*sin(pitch);
    hx=my*cos(roll)+mz*sin(roll);

    %计算与北偏航角
    x=atan2(hy,hx);

    fprintf("hy=%f,hx=%f,x=%f\n",hy,hx,x*180/pi);

    %plot(x,'.b');


    %plot3(data0(1),data0(2),data0(3), '.b');%绘制

    %data2=data2.*((((adj-128).*0.5)./128)+1);%对3轴进行校准
    %%data2(3)=-data2(3);%对Z轴翻转
    %plot3(data2(1),data2(2),data2(3), '.b');%绘制
end

%close(s)