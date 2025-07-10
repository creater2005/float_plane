#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>

/*---------------------------- 数据包格式定义 ----------------------------*/
#define SERIAL_HEADER_1           0x55    // 数据包第一帧头
#define SERIAL_HEADER_2           0xAA    // 数据包第二帧头
#define SERIAL_DATA_MIN           800     // 数据字段最小值
#define SERIAL_DATA_MAX           2200    // 数据字段最大值

// 协议帧头定义
#define PACKET_HEADER_1    0x55    // 帧头第一字节
#define PACKET_HEADER_2    0xAA    // 帧头第二字节

// 数据有效范围
#define PACKET_MIN_VALUE   800     // 数据最小值（单位：微秒/角度等）
#define PACKET_MAX_VALUE   2200    // 数据最大值


/*--------------------------- 数据包结构体定义 ---------------------------*/
#pragma pack(push, 1)  // 确保结构体紧凑排列
typedef struct {
    uint16_t data1;     // 第一个数据值（800-2200）
    uint16_t data2;     // 第二个数据值（800-2200）
    uint8_t  valid;     // 数据有效性标志 (1=有效)
} SerialPacket;
#pragma pack(pop)


void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(void);

/**
  * @brief  获取解析完成的数据包
  * @param  pPacket 数据包存储指针
  * @retval 1:成功获取有效数据包 0:无新数据
  */
uint8_t Serial_GetPacket(SerialPacket *pPacket);

#endif
