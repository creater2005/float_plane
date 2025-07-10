#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>

/*---------------------------- ���ݰ���ʽ���� ----------------------------*/
#define SERIAL_HEADER_1           0x55    // ���ݰ���һ֡ͷ
#define SERIAL_HEADER_2           0xAA    // ���ݰ��ڶ�֡ͷ
#define SERIAL_DATA_MIN           800     // �����ֶ���Сֵ
#define SERIAL_DATA_MAX           2200    // �����ֶ����ֵ

// Э��֡ͷ����
#define PACKET_HEADER_1    0x55    // ֡ͷ��һ�ֽ�
#define PACKET_HEADER_2    0xAA    // ֡ͷ�ڶ��ֽ�

// ������Ч��Χ
#define PACKET_MIN_VALUE   800     // ������Сֵ����λ��΢��/�Ƕȵȣ�
#define PACKET_MAX_VALUE   2200    // �������ֵ


/*--------------------------- ���ݰ��ṹ�嶨�� ---------------------------*/
#pragma pack(push, 1)  // ȷ���ṹ���������
typedef struct {
    uint16_t data1;     // ��һ������ֵ��800-2200��
    uint16_t data2;     // �ڶ�������ֵ��800-2200��
    uint8_t  valid;     // ������Ч�Ա�־ (1=��Ч)
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
  * @brief  ��ȡ������ɵ����ݰ�
  * @param  pPacket ���ݰ��洢ָ��
  * @retval 1:�ɹ���ȡ��Ч���ݰ� 0:��������
  */
uint8_t Serial_GetPacket(SerialPacket *pPacket);

#endif
