// Deng's FOC 开环速度控制例程 测试库：SimpleFOC 2.1.1 测试硬件：灯哥开源Mini FOC
// 串口中输入"T+数字"设定两个电机的转速，如设置电机以 10rad/s 转动，输入 "T10"，电机上电时默认会以 5rad/s 转动
// 在使用自己的电机时，请一定记得修改默认极对数，即 BLDCMotor(7) 中的值，设置为自己的极对数数字
// 程序默认设置的供电电压为 12V,用其他电压供电请记得修改 voltage_power_supply , voltage_limit 变量中的值

#include <SimpleFOC.h>
#define pwmA 32 
#define pwmB 33
#define pwmC 25
#define enable_pin 12  //使能引脚

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, enable_pin);


//目标变量
float target_velocity = 10;

//串口指令设置
Commander command = Commander(Serial);
void doTarget(char* cmd) {
  command.scalar(&target_velocity, cmd);
}

void setup() {

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);
  motor.voltage_limit = 3;    // [V]
  motor.velocity_limit = 30;  // [rad/s]

  //开环控制模式设置
  motor.controller = MotionControlType::velocity_openloop;

  //初始化硬件
  motor.init();
  
  //增加 T 指令
  command.add('T', doTarget, "target velocity");

  Serial.begin(115200);
  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}

void loop() {
  motor.move(target_velocity);

  //用户通讯
  command.run();
}
