#include <Arduino.h>
#include "SX126X_Driver.h"
#include "SPI.h"
// put function declarations here:
uint8_t TXbuffer[10] = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA};
uint8_t RXbuffer[10] = {0};
uint32_t Fre[5] = {470800000, 480570000, 492570000, 500000000, 510000000};
// long SysTick = 0;
uint16_t T_Cnt = 0;
uint16_t R_Cnt = 0;
uint16_t E_Cnt = 0;
uint8_t SF;
unsigned int BW;
unsigned int frq;
int8_t PktSnr_Value = 0; // SNR
int8_t SNR = 0;
int8_t PktRssi_Value = 0; // rssi
int8_t RSSI = 0;
int8_t Avg_RSSI = 0;
uint8_t communication_states;
#define VSPI_MISO MISO
#define VSPI_MOSI MOSI
#define VSPI_SCLK SCK
#define VSPI_SS SS
void setup()
{
  Serial.begin(115200);
  Serial.println("system begin");
  // put your setup code here, to run once:
  // 配置各个参数
  G_LoRaConfig.LoRa_Freq = Fre[0];                // 中心频率
  G_LoRaConfig.BandWidth = LORA_BW_125;           // BW = 125KHz  BW125KHZ
  G_LoRaConfig.SpreadingFactor = LORA_SF9;        // SF = 9
  G_LoRaConfig.CodingRate = LORA_CR_4_6;          // CR = 4/6
  G_LoRaConfig.PowerCfig = 22;                    // 输入范围：-3~22，根据实际使用硬件LSD4RF-2R717N30或LSD4RF-2R722N20选择SX126xSetTxParams函数
  G_LoRaConfig.HeaderType = LORA_PACKET_EXPLICIT; // 包头格式设置，显性包头：LORA_PACKET_EXPLICIT；隐性包头：LORA_PACKET_IMPLICIT
  // 若设置为显性包头，发送端将会将PalyLoad长度、编码率、CRC等加入到包头中发送给接收端

  G_LoRaConfig.CrcMode = LORA_CRC_ON;     // CRC校验开启：LORA_CRC_ON，关闭：LORA_CRC_OFF
  G_LoRaConfig.InvertIQ = LORA_IQ_NORMAL; // IQ信号格式，LORA_IQ_NORMAL：标准模式，LORA_IQ_INVERTED：反转模式；
  G_LoRaConfig.PreambleLength = 8;        // 前导码长度
  G_LoRaConfig.PayloadLength = 10;        // 数据包长度
  if (SX126x_Lora_init() != NORMAL)       // 无线模块初始化
  {
    while (1)
    {
      Serial.printf("SX126X init ERROR ::::%d!!!\n", SX126x_Lora_init());
    }
  }
  else
  {
    Serial.printf("SX126x_Lora_init finished\n");
  }
}
uint8_t c = 0xaa;
void loop()
{
  // put your main code here, to run repeatedly:
  // SPI.transfer(0x55);
  // SX126X_TxPacket(&c);
  delay(10);
}