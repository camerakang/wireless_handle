/**
 ******************************************************************************
 * 文件名 ：   SX127X_HAL.c
 * 作者   ：   LSD RF Team
 * 版本   ：   V1.0.0
 * 时间   ：   15-Aug-2018
 * 文件描述：
 *     该文件为SX127X模块的硬件层，包含MCU与SX127X模块的SPI配置，GPIO口初始化，以
 *及用于SX127X寄存器、FIFO读写操作；
 *    客户在使用模块时候需要移植该文件，保证各个函数名、函数形参不变的情况下
 *根据自己的MCU平台修改函数内容，使各个功能块正常运行。硬件层占用资源如下：
 *
 *SPI：本例程使用STM32L4的SPI3进行与SX127X模块通信。
 *GPIO口：本例程使用的GPIO口详情如下：
 *        PB1  ---> DIO1
 *        PC4  ---> DIO2
 *        PB2  ---> Busy
 *        PB0  ---> SW2
 *        PC5  ---> SW1
 *        PA7  ---> RST
 *        PA15 ---> NSS
 *        PC12 ---> M0SI
 *        PC11 ---> MISO
 *        PC10 ---> SCK
 *******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "SX126X_Hal.h"
#include "SPI.h"
#include "esp32-hal-gpio.h"
#include "Arduino.h"
// SPI_HandleTypeDef SPI3_InitStruct;
//-----------------------------GPIO-----------------------------//
// 该部分函数为系统用到的GPIO的初始化函数，用户根据自己的平台相应修改
//--------------------------------------------------------------//
void ISR()
{
  Serial.println("ISR");
}
/**
 * @简介：该函数为DIO1输入初始化及中断、优先级配置；
 * @参数：无
 * @返回值：无
 */

void SX126X_DIO1_INPUT(void)
{
  pinMode(GPIO_NUM_8, INPUT_PULLDOWN); // 将引脚设置为输入模式，并启用下拉电阻
  attachInterrupt(digitalPinToInterrupt(GPIO_NUM_8), ISR, RISING);
}
/**
 * @简介：该函数为DIO1输入中断开启使能；
 * @参数：无
 * @返回值：无
 */
void interruptHandler(void)
{
  Serial.println("Triggered interrupt!");
}
void SX126X_DIO1_INTENABLE(void)
{
  // NVIC_EnableIRQ(EXTI1_IRQn);
  attachInterrupt(digitalPinToInterrupt(GPIO_NUM_8), ISR, RISING);
}
/**
 * @简介：该函数为DIO1输入中断关闭使能；
 * @参数：无
 * @返回值：无
 */
void SX126X_DIO1_INTDISABLE(void)
{
  detachInterrupt(GPIO_NUM_8);
}
/**
 * @简介：该函数为DIO1输入状态获取；
 * @参数：无
 * @返回值：DIO1状态"1"or"0"
 */
bool SX126X_DIO1_GetState(void)
{
  bool State;
  State = digitalRead(GPIO_NUM_8);
  return State;
}

/**
 * @简介：该函数为DIO2输入初始化及中断、优先级配置；
 * @参数：无
 * @返回值：无
 */
void SX126X_DIO2_INPUT(void)
{
  Serial.println("SX126X_DIO2_INPUT");
}
/**
 * @简介：该函数为DIO2输入中断开启使能；
 * @参数：无
 * @返回值：无
 */
void SX126X_DIO2_INTENABLE(void)
{
  Serial.println("SX126X_DIO2_INTENABLE");
}
/**
 * @简介：该函数为DIO2输入中断关闭使能；
 * @参数：无
 * @返回值：无
 */
void SX126X_DIO2_INTDISABLE(void)
{
  Serial.println("SX126X_DIO2_INTDISABLE");
}
/**
 * @简介：该函数为DIO2输入状态获取；
 * @参数：无
 * @返回值：DIO2状态"1"or"0"
 */
bool SX126X_DIO2_GetState(void)
{
  Serial.println("SX126X_DIO2_GetState");
}
/**
 * @简介：该函数为Busy输入初始化及中断、优先级配置；
 * @参数：无
 * @返回值：无
 */
void SX126X_Busy_INPUT(void)
{
  pinMode(GPIO_NUM_9, INPUT);
}
/**
 * @简介：该函数为Busy输入中断开启使能；
 * @参数：无
 * @返回值：无
 */

void SX126X_Busy_INTENABLE(void)
{
  // HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  attachInterrupt(digitalPinToInterrupt(GPIO_NUM_9), ISR, RISING);
}
/**
 * @简介：该函数为Busy输入中断关闭使能；
 * @参数：无
 * @返回值：无
 */
void SX126X_Busy_INTDISABLE(void)
{
  // HAL_NVIC_DisableIRQ(EXTI2_IRQn);
  detachInterrupt(digitalPinToInterrupt(GPIO_NUM_9));
}
/**
 * @简介：该函数为Busy输入状态获取；
 * @参数：无
 * @返回值：Busy状态"1"or"0"
 */
bool SX126X_Busy_GetState(void)
{
  bool State;
  State = digitalRead(GPIO_NUM_9);
  return State;
}
/**
 * @简介：该函数为Busy等待；
 * @参数：无
 * @返回值：无
 */
void SX126xWaitOnBusy(void)
{
  // Serial.println("SX126xWaitOnBusy");
  SX126X_Busy_INPUT();
  while (SX126X_Busy_GetState() == 1)
  {
    Serial.println("SX126xWaitOnBusy waitting");
  }
}
/**
 * @简介：该函数为高频开关SWCTL2输出控制；
 * @参数：PinState为"1"表示输出高电平，"0"输出低电平；
 * @返回值：无
 */
void SX126X_SWCTL2_OUTPUT(bool PinState)
{
  Serial.println("SX126X_SWCTL2_OUTPUT");
}
/**
 * @简介：该函数为高频开关SWCTL1输出控制；
 * @参数：PinState为"1"表示输出高电平，"0"输出低电平；
 * @返回值：无
 */
void SX126X_SWCTL1_OUTPUT(bool PinState)
{
  Serial.println("SX126X_SWCTL1_OUTPUT");
}

/**
 * @简介：普通IO口，监控使用
 * @参数：PinState为"1"表示输出高电平，"0"输出低电平；
 * @返回值：无
 */
void IO_SET(bool PinState)
{
  Serial.println("IO_SET");
}
/**
 * @简介：该函数为SPI的片选引脚NSS输出控制；
 * @参数：PinState为"1"表示输出高电平，"0"输出低电平；
 * @返回值：无
 */
void SX126X_NSS_OUTPUT(bool PinState)
{
  pinMode(SS, OUTPUT);
  digitalWrite(SS, PinState);
}
/**
 * @简介：该函数为SX126X复位引脚NRST输出控制；
 * @参数：PinState为"1"表示输出高电平，"0"输出低电平；
 * @返回值：无
 */
void SX126X_RESET_OUTPUT(bool PinState)
{
  pinMode(GPIO_NUM_14, PULLUP);
  digitalWrite(GPIO_NUM_14, PinState);
}
//-----------------------------SPI-----------------------------//
// 该部分函数为MCU对SX127X模块SPI通信部分，包含SPI口及配置初始化
//--------------------------------------------------------------//

/**
 * @简介：该函数用于MCU对SPI对应IO口初始化；
 * @参数：无
 * @返回值：无
 */
void SX126X_SPIGPIO_Init(void)
{

  pinMode(GPIO_NUM_10, OUTPUT);
  pinMode(GPIO_NUM_12, PULLUP); // 将引脚设置为输出模式
  // pinMode(GPIO_NUM_13, OUTPUT);    // 将引脚设置为输出模式
  pinMode(GPIO_NUM_11, PULLUP); // 将引脚设置为输出模式
}
/**
 * @简介：该函数用于MCU对SPI配置初始化；
 * @参数：无
 * @返回值：无
 */
void SX126X_SPI_Init(void)
{
  SPI.begin(SCK, MISO, MOSI, SS);
  pinMode(SS, OUTPUT);
  SPI.beginTransaction(SPISettings(1000000, SPI_MSBFIRST, SPI_MODE0));
}
/**
 * @简介：SX126X  向寄存器地址连续发送数据
 * @参数：uint8_t addr,寄存器地址 uint8_t *buffer,发送数组指针 uint8_t size指针长度
 * @返回值：无
 */
uint8_t SX126X_ReadWriteByte(uint8_t data)
{
  uint8_t RxDat;

  RxDat = SPI.transfer(data);
  return RxDat;
}
