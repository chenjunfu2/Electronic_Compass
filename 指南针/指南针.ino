/*
 Name:		指南针.ino
 Created:	2024/6/6 0:56:29
 Author:	chenj
*/

//#include <SimpleFOC.h>
//#include <math.h>
//#include "MPU9250\MPU6050.h"
//
////mpu9250传感器
//MPU6050 mpu9250;
//
////AS5600编码器，获取电机位置
//MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
//
///*
//PWM1: 连接到 D3（GPIO0）
//PWM2: 连接到 D4（GPIO2）
//PWM3: 连接到 D5（GPIO14）
//EN 引脚: 连接到 D8（GPIO15）
//*/
//// BLDC motor & driver instance
//BLDCMotor motor = BLDCMotor(7);//2804电机的极对数为7
//BLDCDriver3PWM driver = BLDCDriver3PWM(0, 2, 14, 15);//PWM引脚，foc板与esp板接线要对应
//
//
////mpu9250传感器
//#define D_SDA_PIN 4							// SDA引脚，默认gpio4(D2)
//#define D_SCL_PIN 5							// SCL引脚，默认gpio5(D1)
//
////电机编码器
//#define T_SDA_PIN 12						// SDA引脚，默认gpio12(D6)
//#define T_SCL_PIN 13						// SCL引脚，默认gpio13(D7)
//
//// the setup function runs once when you press reset or power the board
//void setup() {
//	Serial.begin(115200);//esp8266默认频率，这样不会乱码
//	Serial.flush();
//
//	while (true)
//	{
//		if (Serial.available() > 0 && Serial.read() == 'S')
//		{
//			break;
//		}
//		delay(10);
//	}
//
//	//设置i2c引脚初始化编码器
//	Wire.begin(T_SDA_PIN, T_SCL_PIN);
//	sensor.init();//传感器初始化
//	motor.linkSensor(&sensor);//电机连接传感器
//
//	driver.voltage_power_supply = 18.85;//驱动器连接的电源电压
//	driver.init();//驱动器初始化
//	motor.linkDriver(&driver);//电机连接驱动器
//
//	motor.foc_modulation = FOCModulationType::SpaceVectorPWM;//采用PWM方式驱动
//	motor.controller = MotionControlType::angle;//弧度模式
//	motor.PID_velocity.P = 0.05f;//速度P值，这个值不能填太大，否则容易抖动
//	motor.PID_velocity.I = 0.02;//这个值越大，响应速度会慢下来
//	motor.PID_velocity.D = 0;
//	motor.voltage_limit = driver.voltage_power_supply / 2.0;//限制电压最大值，这个值一般为电源电压的一半
//	motor.LPF_velocity.Tf = 0.01f;//滤波
//	motor.P_angle.P = 20;//位置PID的P值
//	motor.velocity_limit = 20;//限制最大速度，弧度/秒
//	motor.init();//电机初始化
//	motor.initFOC();//传感器校正和启动FOC
//
//	/*
//	//复用i2c引脚初始化多轴传感器
//	Wire.begin(D_SDA_PIN, D_SCL_PIN);
//	mpu9250.initialize();
//	pinMode(LED_BUILTIN, OUTPUT);//设置自带的蓝灯
//	digitalWrite(LED_BUILTIN, HIGH);//高电平关灯
//
//	if (!mpu9250.testConnection())
//	{
//		Serial.println("testConnection fail");
//	}
//
//	if (!mpu9250.testMagConnection())
//	{
//		Serial.println("testMagConnection fail");
//	}
//
//	if (!mpu9250.testMagnetometer())
//	{
//		Serial.println("testMagnetometer fail");
//	}
//
//	Serial.printf("S\n");
//
//	delay(100);
//
//	//校准磁力计
//	//mpu9250.MagCalibrateTimeVersion();
//	//方便调试，直接使用已校准的值
//	//Off:[ 58.500000 46.000000 135.000000 ]
//	//Scl:[ 0.992862 1.068356 0.946259 ]
//	mpu9250.setUserOff(58.500000, 46.000000, 135.000000);
//	mpu9250.setUserScl(0.992862, 1.068356, 0.946259);
//
//	Wire.begin(T_SDA_PIN, T_SCL_PIN);
//	*/
//	delay(1000);
//	return;
//}
//
//long double GetYaw2North(void)//返回弧度制，x*180.0/PI转换到角度制
//{
//	//测量顺序不要换，上面的延迟高，下面的低，反过来可能导致需要低间隔的数据测量时间间隔过高
//	long double mx, my, mz;
//	mpu9250.getMagnetometer(&mx, &my, &mz);//读出数据已完成校准
//	int16_t ax, ay, az;
//	mpu9250.getAcceleration(&ax, &ay, &az);
//	//对速度影响较大的输出放最后，减少测量间隔
//	Serial.printf("[Magnetometer]x=%lf,y=%lf,z=%lf  [Acceleration]x=%d,y=%d,z=%d\n", mx, my, mz, ax, ay, az);
//
//	//水平计算
//	long double roll = atan2((long double)ay, (long double)az);
//	long double pitch = atan2((long double)ax, (long double)az);
//
//	//倾斜校准
//	long double hy = mx * cos(pitch) + my * sin(roll) * sin(pitch) - mz * cos(roll) * sin(pitch);
//	long double hx = my * cos(roll) + mz * sin(roll);
//
//	//计算水平与北偏航角
//	long double x = atan2(hy, hx);
//
//	Serial.printf("[roll=%lf,pitch=%lf,hy=%lf,hx=%lf]x=%lf\n", roll * 180.0 / PI, pitch * 180.0 / PI, hy * 180.0 / PI, hx * 180.0 / PI, x * 180.0 / PI);
//
//	return x;
//}
//
//
//// the loop function runs over and over again until power down or reset
//unsigned long ms = 0;
//
//void loop(void)
//{
//	motor.loopFOC();
//
//	if ((millis() - ms) > 10)
//	{
//		if (Serial.available())
//		{
//			motor.move(Serial.readString().toFloat() * PI / 180);//转到弧度
//		}
//
//		//Wire.begin(D_SDA_PIN, D_SCL_PIN);
//		//long double x = GetYaw2North() * 180.0 / PI;//获取磁力计数据
//		//Wire.begin(T_SDA_PIN, T_SCL_PIN);
//		//motor.move(x);//转到弧度
//
//		ms = millis();
//	}
//}

	/*

	Serial.printf("S\n");

	delay(100);


#pragma pack(push,1)
	struct
	{
		int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
	}data;

	struct
	{
		uint8_t jx, jy, jz;
	}adj;
#pragma pack(pop)
	
	
	mpu9250.getAdjustment(&adj.jx, &adj.jy, &adj.jz);
	Serial.write((uint8_t *)&adj, sizeof(adj));
	
	delay(100);

	while (true)
	{
		mpu9250.getMotion6(&data.ax, &data.ay, &data.az, &data.gx, &data.gy, &data.gz);
		mpu9250.getMagnetometerRaw(&data.mx, &data.my, &data.mz);

		Serial.write((uint8_t *)&data, sizeof(data));
		delay(100);
	}

	*/


	/**
	 *
	 * Position/angle motion control example
	 * Steps:
	 * 1) Configure the motor and magnetic sensor
	 * 2) Run the code
	 * 3) Set the target angle (in radians) from serial terminal
	 *
	 */


#include <SimpleFOC.h>

//#define MPU

#ifdef MPU
#include <math.h>
#include "MPU9250\MPU6050.h"

//mpu9250传感器
MPU6050 mpu9250;
bool bMpu9250_OK = true;
#endif // MPU


#define D_SDA_PIN 4		// SDA引脚，默认gpio4(D2)
#define D_SCL_PIN 5		// SCL引脚，默认gpio5(D1)

#define VOLTAGE_POWER_SUPPLY 12.0//电源电压

//I2C
TwoWire I2Cone{};

//驱动板AS5600
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);//2804电机的极对数为7

//PWM1: 连接到 D5（GPIO14）
//PWM2: 连接到 D6（GPIO12）
//PWM3: 连接到 D7（GPIO13）
//EN  : 连接到 D8（GPIO15）
BLDCDriver3PWM driver = BLDCDriver3PWM(14, 12, 13, 15);//前三个分别对应3个PWM引脚，最后一个是使能引脚

//电机角度
float x = 0;//X

//显示编码器信息
bool bShowSensor = false;

//设置FOC开关状态
bool bEnableFOC = true;

//设置回显
bool bEcho = true;

//调参用命令

void to_bool(Commander &cmder, bool *value, char *user_cmd)
{
	bool GET = cmder.isSentinel(user_cmd[0]);
	if (!GET) *value = atoi(user_cmd);
	if (!cmder.com_port || cmder.verbose == VerboseMode::nothing) return;
	cmder.com_port->println(*value);
}


//命令（换行结束，回显打开）
Commander command0(Serial, '\n', bEcho);//O
Commander command1(Serial, '\n', bEcho);//O
Commander command2(Serial, '\n', bEcho);//O

void Cmd_S(char *cmd)
{
	to_bool(command0, &bShowSensor, cmd);
}
void Cmd_E(char *cmd)
{
	bool bLast = bEnableFOC;
	to_bool(command0, &bEnableFOC, cmd);
	if (bLast != bEnableFOC)
	{
		if (bEnableFOC)
		{
			driver.enable();
		}
		else
		{
			driver.disable();
		}
	}
}
void Cmd_O(char *cmd)
{
	bool bLast = bEcho;
	to_bool(command0, &bEcho, cmd);
	if (bLast != bEcho)
	{
		command0.echo = bEcho;
		command1.echo = bEcho;
		command2.echo = bEcho;

		VerboseMode bVb = (bEcho == true) ? VerboseMode::user_friendly : VerboseMode::nothing;

		command0.verbose = bVb;
		command1.verbose = bVb;
		command2.verbose = bVb;
	}
}
void Cmd_G(char *cmd)
{
	//输出所有参数
	Serial.printf(
		"=============================Value=============================\n"\
		"velocity P,I,D,F: %.6lf, %.6lf, %.6lf, %.6lf\n"\
		"angle    P,I,D,F: %.6lf, %.6lf, %.6lf, %.6lf\n"\
		"velocity limit: %.6lf, current x: %.6lf\n"\
		"ShowSensor[%d], EnableFOC[%d], Echo[%d]\n"\
		"===============================================================\n",
		motor.PID_velocity.P, motor.PID_velocity.I, motor.PID_velocity.D, motor.LPF_velocity.Tf,
		motor.P_angle.P, motor.P_angle.I, motor.P_angle.D, motor.LPF_angle.Tf,
		motor.velocity_limit, x * 180.0 / PI,//x要转换为角度输出
		bShowSensor, bEnableFOC, bEcho);
}

void Cmd_X(char *cmd)
{
	float tmp;
	command0.scalar(&tmp, cmd);
	x = tmp * PI / 180.0;//输入转换为弧度存入x
}
void Cmd_L(char *cmd)
{
	command0.scalar(&motor.velocity_limit, cmd);
}

void Cmd_V(char *cmd)
{
	command1.run(cmd);
}
void Cmd_VP(char *cmd)
{
	command1.scalar(&motor.PID_velocity.P, cmd);
}
void Cmd_VI(char *cmd)
{
	command1.scalar(&motor.PID_velocity.I, cmd);
}
void Cmd_VD(char *cmd)
{
	command1.scalar(&motor.PID_velocity.D, cmd);
}
void Cmd_VF(char *cmd)
{
	command1.scalar(&motor.LPF_velocity.Tf, cmd);
}


void Cmd_A(char *cmd)
{
	command2.run(cmd);
}
void Cmd_AP(char *cmd)
{
	command2.scalar(&motor.P_angle.P, cmd);
}
void Cmd_AI(char *cmd)
{
	command2.scalar(&motor.P_angle.I, cmd);
}
void Cmd_AD(char *cmd)
{
	command2.scalar(&motor.P_angle.D, cmd);
}
void Cmd_AF(char *cmd)
{
	command2.scalar(&motor.LPF_angle.Tf, cmd);
}

//初始化
void setup()
{
	delay(5000);
	Serial.begin(115200);//esp8266默认频率，这样不会乱码
	Serial.flush();
	Serial.println("Test.");

	//等待开始指令
	while (true)
	{
		if (Serial.available() > 0)
		{
			int read = Serial.read();
			if (read == (int)'S')
			{
				break;
			}
			else
			{
				Serial.printf("read:%c[%d]\n", (char)read, read);
			}
		}
		delay(10);
	}
	Serial.println("S");

	//设置wire
	//Wire.begin(D_SDA_PIN, D_SCL_PIN);
	I2Cone.begin(D_SDA_PIN, D_SCL_PIN);//400000UL

#ifdef MPU
	//初始化mpu9250
	mpu9250.initialize();
	if (!mpu9250.testConnection())
	{
		Serial.println("testConnection fail");
		bMpu9250_OK = false;
	}

	if (!mpu9250.testMagConnection())
	{
		Serial.println("testMagConnection fail");
		bMpu9250_OK = false;
	}

	if (!mpu9250.testMagnetometer())
	{
		Serial.println("testMagnetometer fail");
		bMpu9250_OK = false;
	}

	//校准磁力计
	//mpu9250.MagCalibrateTimeVersion();
	//方便调试，直接使用已校准的值
	//Off:[ 58.500000 46.000000 135.000000 ]
	//Scl:[ 0.992862 1.068356 0.946259 ]
	mpu9250.setUserOff(58.500000, 46.000000, 135.000000);
	mpu9250.setUserScl(0.992862, 1.068356, 0.946259);
#endif

	//初始化foc和电机编码器等
// initialise magnetic sensor hardware
	sensor.init(&I2Cone);//传感器初始化
	// link the motor to the sensor
	motor.linkSensor(&sensor);//电机连接传感器

	// driver config
	// power supply voltage [V]
	driver.voltage_power_supply = VOLTAGE_POWER_SUPPLY;//驱动器连接的电源电压
	driver.voltage_limit = VOLTAGE_POWER_SUPPLY;//驱动器最大电压// / 2.0
	driver.init();//驱动器初始化
	// link the motor and the driver
	motor.linkDriver(&driver);//电机连接驱动器

	// choose FOC modulation (optional)
	motor.foc_modulation = FOCModulationType::SpaceVectorPWM;//采用PWM方式驱动

	// set motion control loop to be used
	motor.controller = MotionControlType::angle;//角度模式

	// set limit value
	motor.voltage_sensor_align = 1.0;//对齐电压
	motor.voltage_limit = VOLTAGE_POWER_SUPPLY / 2.0;//电机电压最大值，这个值一般为电源电压的一半
	//motor.current_limit = 0.5;//电流限制
	motor.velocity_limit = 40.0;//限制最大速度，弧度/秒//L
	
	// velocity PI controller parameters
	motor.PID_velocity.P = 0.05;//速度P值，反应速度//VP
	motor.PID_velocity.I = 0.02;//速度I值，误差累计//VI
	motor.PID_velocity.D = 0.0;//速度D值//VD
	motor.LPF_velocity.Tf = 0.05;//速度低通滤波时间常数//VF

	// angle P controller
	motor.P_angle.P = 20.0;//角度P值//AP
	motor.P_angle.I = 0.0;//角度I值//AI
	motor.P_angle.D = 0.0;//角度D值//AD
	motor.LPF_angle.Tf = 0.00;//角度低通滤波时间常数//AF

	// comment out if not needed
	motor.useMonitoring(Serial);

	// initialize motor
	motor.init();//电机初始化
	// align sensor and start FOC
	motor.initFOC();//传感器校正和启动FOC

	Serial.println("Motor ready.");

	//注册命令

	//控制命令
	command0.add('S', Cmd_S, "Set angle show (bool:0/1)");						//设置角度输出
	command0.add('E', Cmd_E, "Set FOC enable (bool:0/1)");						//设置FOC使能
	command0.add('O', Cmd_O, "Set Echo (bool:0/1)");							//设置命令回显
	command0.add('G', Cmd_G, "Get All Value (void)");							//获取所有参数

	//设置命令
	command0.add('X', Cmd_X, "Set motor angle (float)");						//设置电机角度
	command0.add('L', Cmd_L, "Set velocity limit (float)");						//设置最大速度
	
	//二级设置命令
	command0.add('V', Cmd_V, "Set velocity P/I/D/F (use VP/VI/VD/VF + float)");		//设置速度PID、滤波
	command1.add('P', Cmd_VP, "Set velocity P (float)");
	command1.add('I', Cmd_VI, "Set velocity I (float)");
	command1.add('D', Cmd_VD, "Set velocity D (float)");
	command1.add('F', Cmd_VF, "Set velocity low pass filtering time constant (float)");	//设置速度低通滤波

	command0.add('A', Cmd_A, "Set angle P/I/D/F (use AP/AI/AD/AF + float)");			//设置角度PID、滤波
	command2.add('P', Cmd_AP, "Set angle P (float)");
	command2.add('I', Cmd_AI, "Set angle I (float)");
	command2.add('D', Cmd_AD, "Set angle D (float)");
	command2.add('F', Cmd_AF, "Set angle low pass filtering time constant (float)");	//设置角度低通滤波

	_delay(1000);
}

#ifdef MPU
long double GetYaw2North(void)//返回弧度制，x*180.0/PI转换到角度制
{
	//测量顺序不要换，上面的延迟高，下面的低，反过来可能导致需要低间隔的数据测量时间间隔过高
	long double mx, my, mz;
	mpu9250.getMagnetometer(&mx, &my, &mz);//读出数据已完成校准
	int16_t ax, ay, az;
	mpu9250.getAcceleration(&ax, &ay, &az);
	//对速度影响较大的输出放最后，减少测量间隔
	Serial.printf("[Magnetometer]x=%lf,y=%lf,z=%lf  [Acceleration]x=%d,y=%d,z=%d\n", mx, my, mz, ax, ay, az);

	//水平计算
	long double roll = atan2((long double)ay, (long double)az);
	long double pitch = atan2((long double)ax, (long double)az);

	//倾斜校准
	long double hy = mx * cos(pitch) + my * sin(roll) * sin(pitch) - mz * cos(roll) * sin(pitch);
	long double hx = my * cos(roll) + mz * sin(roll);

	//计算水平与北偏航角
	long double x = atan2(hy, hx);

	Serial.printf("[roll=%lf,pitch=%lf,hy=%lf,hx=%lf]x=%lf\n", roll * 180.0 / PI, pitch * 180.0 / PI, hy * 180.0 / PI, hx * 180.0 / PI, x * 180.0 / PI);

	return x;
}
#endif

void loop()
{
	//if ((millis() - ms) > 10)
	//{
	//	//sensor.update();
	//	Serial.print(sensor.getAngle() * 180 / PI);
	//	Serial.print("\t");
	//	Serial.println(sensor.getVelocity() * 180 / PI);
	//
	//	if (bMpu9250_OK != false)
	//	{
	//		x = GetYaw2North();
	//	}
	//	else
	//	{
	//		if (Serial.available() > 0)
	//		{
	//			x = Serial.readString().toFloat() * PI / 180.0;
	//		}
	//	}
	//
	//	ms = millis();
	//}

	if (bEnableFOC == true)
	{
		motor.loopFOC();//foc循环
		motor.move(x);//转到弧度
	}
	if (bShowSensor == true)
	{
		if (bEnableFOC == false)
		{
			sensor.update();//如果foc被关了，需要自己更新，否则foc会更新，只要输出即可
		}
		Serial.printf("A:%.3f,V:%.3f\n", sensor.getAngle(), sensor.getVelocity());//显示当前角度//S
	}
	command0.run();//执行命令(从Serial读取)
}
