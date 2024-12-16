s = serialport("COM9",115200,"Timeout",10);
s.flush();

write(s,'S',"char");%д�뿪ʼ����

readline(s);%��ȡ���׵���һ�еĳ�ʼ�������Ϣ

adj=read(s,3,"uint8");%��ȡ3��У׼ֵ
fprintf("adj:%d,%d,%d",adj(1),adj(2),adj(3));



%file=fopen('write.bin','wb');
%fwrite(file,adj,"uint8");%д���ļ�
%
%while true
%    data=read(s,9,"int16");
%    fwrite(file,data,"int16");
%end



%��ͼ׼��
clf;
%hold on;
%grid on;
%axis equal;
%xlabel('X��');
%ylabel('Y��');
%zlabel('Z��');

%ʹ��3��������Ϊ�ռ�����

while true
    data0=read(s,3,"int16");%���ٶȼ�
    data1=read(s,3,"int16");%������
    data2=read(s,3,"int16");%������
    
    ax=data0(1);
    ay=data0(2);
    az=data0(3);

    %ˮƽ����
    roll=atan2(ay,az);
    pitch=atan2(ax,az);

    %roll=atan2(ay,sqrt(ax*ax+az*az));
    %pitch=atan2(ax,sqrt(ay*ay+az*az));

    fprintf("roll=%f,pitch=%f,",roll*180/pi,pitch*180/pi);


    mx=data2(1);
    my=data2(2);
    mz=data2(3);

    %����������У׼
    mx=mx*((((adj(1)-128)*0.5)/128)+1);
    my=my*((((adj(2)-128)*0.5)/128)+1);
    mz=mz*((((adj(3)-128)*0.5)/128)+1);


    %У׼ֵ
    VOX=101.09;
    VOY=-2.43;
    VOZ=234.65;

    VRX=269.56;
    VRY=283.29;
    VRZ=242.96;

    %vk=(VRX+VRY+VRZ)/3;
    vk=1;

    %����У׼
    mx=(mx-VOX)/VRX*vk;
    my=(my-VOY)/VRY*vk;
    mz=(mz-VOZ)/VRZ*vk;

    %plot3(mx,my,mz, '.b');%����
    fprintf("mx=%f,my=%f,mz=%f,",mx,my,mz);

    %��бУ׼
    hy=mx*cos(pitch)+my*sin(roll)*sin(pitch)-mz*cos(roll)*sin(pitch);
    hx=my*cos(roll)+mz*sin(roll);

    %�����뱱ƫ����
    x=atan2(hy,hx);

    fprintf("hy=%f,hx=%f,x=%f\n",hy,hx,x*180/pi);

    %plot(x,'.b');


    %plot3(data0(1),data0(2),data0(3), '.b');%����

    %data2=data2.*((((adj-128).*0.5)./128)+1);%��3�����У׼
    %%data2(3)=-data2(3);%��Z�ᷭת
    %plot3(data2(1),data2(2),data2(3), '.b');%����
end

%close(s)