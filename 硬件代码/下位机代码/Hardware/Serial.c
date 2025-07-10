/* serial.c */
#include "stm32f10x.h"
#include "serial.h"
#include <stdio.h>
#include <stdarg.h>

/*------------------------- 私有全局变量定义 -------------------------*/
volatile SerialPacket serial_rx_packet = {0};   // 接收数据包
volatile uint8_t serial_rx_buffer[7];          // 接收缓冲区（2头 + 4数据 + 1校验）
volatile uint8_t serial_rx_index = 0;          // 接收字节索引
volatile uint8_t serial_rx_status = 0;         // 状态机状态（0:等待头 1:头1通过 2:接收数据）

/*------------------------- 私有函数声明 -------------------------*/
static uint8_t CalculateChecksum(uint8_t *data, uint8_t length);

/**
  * @brief  串口2初始化
  * @param  无
  * @retval 无
  */
void Serial_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    /* 使能时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* 配置TX(PA2)为复用推挽输出 */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* 配置RX(PA3)为上拉输入 */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2参数配置 */
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART2, &USART_InitStruct);

    /* 使能接收中断 */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    /* NVIC配置 */
    NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    /* 启动串口 */
    USART_Cmd(USART2, ENABLE);
}

/**
  * @brief  USART2中断服务函数
  * @note   实现协议解析状态机
  */
void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        uint8_t rx_byte = USART_ReceiveData(USART2);
        
        switch(serial_rx_status)
        {
        case 0: // 等待第一个包头
            if(rx_byte == PACKET_HEADER_1)
            {
                serial_rx_buffer[0] = rx_byte;
                serial_rx_index = 1;
                serial_rx_status = 1;
            }
            break;
            
        case 1: // 等待第二个包头
            if(rx_byte == PACKET_HEADER_2)
            {
                serial_rx_buffer[1] = rx_byte;
                serial_rx_index = 2;
                serial_rx_status = 2;
            }
            else
            {
                serial_rx_status = 0; // 包头不匹配，重置状态机
            }
            break;
            
        case 2: // 接收数据部分
            serial_rx_buffer[serial_rx_index++] = rx_byte;
            
            // 接收完成（2头 + 4数据 + 1校验 = 7字节）
            if(serial_rx_index >= 7)
            {
                // 校验和验证（前6字节的累加和）
                uint8_t calc_sum = CalculateChecksum(serial_rx_buffer, 6);
                uint8_t recv_sum = serial_rx_buffer[6];
                
                if(calc_sum == recv_sum)
                {
                    // 大端格式解析数据
                    uint16_t data1 = (serial_rx_buffer[2] << 8) | serial_rx_buffer[3];
                    uint16_t data2 = (serial_rx_buffer[4] << 8) | serial_rx_buffer[5];
                    
                    // 数据范围校验
                    if(data1 >= PACKET_MIN_VALUE && data1 <= PACKET_MAX_VALUE &&
                       data2 >= PACKET_MIN_VALUE && data2 <= PACKET_MAX_VALUE)
                    {
                        serial_rx_packet.data1 = data1;
                        serial_rx_packet.data2 = data2;
                        serial_rx_packet.valid = 1;
                    }
                }
                
                // 重置状态机
                serial_rx_index = 0;
                serial_rx_status = 0;
            }
            break;
            
        default:
            serial_rx_status = 0;
            break;
        }
        
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

/**
  * @brief  计算校验和
  * @param  data 数据指针
  * @param  length 数据长度
  * @retval 校验和
  */
static uint8_t CalculateChecksum(uint8_t *data, uint8_t length)
{
    uint8_t sum = 0;
    for(uint8_t i=0; i<length; i++)
    {
        sum += data[i];
    }
    return sum;
}

/**
  * @brief  获取有效数据包（原子操作）
  * @param  packet 数据包存储指针
  * @retval 1:成功获取 0:无新数据
  */
uint8_t Serial_GetPacket(SerialPacket *packet)
{
    uint8_t result = 0;
    __disable_irq(); // 禁止中断确保原子操作
    if(serial_rx_packet.valid)
    {
        *packet = serial_rx_packet;
        serial_rx_packet.valid = 0;
        result = 1;
    }
    __enable_irq();
    return result;
}

/*---------------------------- 发送功能 ----------------------------*/
/**
  * @brief  发送单个字节
  */
void Serial_SendByte(uint8_t Byte)
{
    USART_SendData(USART2, Byte);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

/**
  * @brief  发送数组
  */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    while(Length--)
    {
        Serial_SendByte(*Array++);
    }
}

/**
  * @brief  发送字符串
  */
void Serial_SendString(char *String)
{
    while(*String != '\0')
    {
        Serial_SendByte(*String++);
    }
}

/**
  * @brief  printf重定向
  */
int fputc(int ch, FILE *f)
{
    Serial_SendByte(ch);
    return ch;
}

/**
  * @brief  自定义格式化输出
  */
void Serial_Printf(char *format, ...)
{
    char buffer[128];
    va_list args;
    
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    Serial_SendString(buffer);
}
