/**
 *
 * Position/angle motion control example
 * Steps:
 * 1) Configure the motor and magnetic sensor
 * 2) Run the code
 * 3) Set the target angle (in radians) from serial terminal
 *
 */
#include <SimpleFOC.h>//一定要加载这个库

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);//采用I2C通信，将AS5600的SDA、SCL引脚与ESP32对应的SDA、SCL连接，就无须定义引脚了，如果要用其他引脚也可以，但是需要重新定义

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);//2804电机的极对数为7
BLDCDriver3PWM driver = BLDCDriver3PWM(25,32,33,12);//PWM引脚为25，32，33，使能引脚为12，foc板与esp板接线要对应

float target_angle = 0;//定义角度变量，也可以是扭矩，也可以是速度
// instantiate the commander
Commander command = Commander(Serial);//使用串口发送命令
void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }

void setup() {//初始化

  // initialise magnetic sensor hardware
  sensor.init();//传感器初始化
  // link the motor to the sensor
  motor.linkSensor(&sensor);//电机连接传感器

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;//驱动器连接的电源电压
  driver.init();//驱动器初始化
  // link the motor and the driver
  motor.linkDriver(&driver);//电机连接驱动器

  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;//采用PWM方式驱动

  // set motion control loop to be used
  motor.controller = MotionControlType::angle;//角度模式

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.05f;//速度P值，这个值不能填太大，否则容易抖动
  motor.PID_velocity.I = 0.02;//这个值越大，响应速度会慢下来
  motor.PID_velocity.D = 0;
  // maximal voltage to be set to the motor
  motor.voltage_limit = 6;//限制电压最大值，这个值一般为电源电压的一半

  // velocity low pass filtering time constant
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01f;//滤波

  // angle P controller
  motor.P_angle.P = 20;//位置PID的P值
  // maximal velocity of the position control
  motor.velocity_limit = 20;//限制最大速度，弧度/秒

  // use monitoring with serial
  Serial.begin(115200);//打开串口
  // comment out if not needed
  motor.useMonitoring(Serial);//


  // initialize motor
  motor.init();//电机初始化
  // align sensor and start FOC
  motor.initFOC();//传感器校正和启动FOC

  // add target command T
  command.add('T', doTarget, "target angle");//通过串口T命令发送位置，比如T6.28,表示电机转6.28弧度,即1圈

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));
  _delay(1000);
}


void loop() {
  motor.loopFOC();//启动，使上劲

  motor.move(target_angle);//转到目标位置

  command.run();//监控输入的命令
}
