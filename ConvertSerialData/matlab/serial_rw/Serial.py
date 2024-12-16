import serial
 
ser = serial.Serial('COM9', 115200, timeout=1)  # 根据实际情况修改串口名称和波特率
file_path = r"data.txt"# 更改为保存数据的文件路径
file = open(file_path, "wb")
 
# 发送初始化字符
while ser.write('S'.encode())!=1:
    continue

 
while True:
    # 读取串口数据
    data = ser.read()
 
    if data:
        # 将字节数据转换为字符串并打印
        file.write(data)
        file.flush()
