#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "Key.h"
#include "PWM.h"
#include "Serial.h"

int main(void)
{          
    PWM_Init();             // 初始化PWM模块
    Serial_Init();          //      初始化串口及协议解析
    
    // 初始化电机中位信号
    PWM_SetPulseWidth_CH3(1500);
    PWM_SetPulseWidth_CH4(1500);
    Delay_ms(2000);         // 等待电调初始化完成

    SerialPacket packet;
    while(1)
    {
		PWM_SetPulseWidth_CH3(2000);
		PWM_SetPulseWidth_CH4(2000);
		if(Serial_GetPacket(&packet)) 
        {
            //if(packet.valid)  
            //{
                // 安全设置PWM值
                uint16_t safe_data1 = (packet.data1 < 800) ? 800 : (packet.data1 > 2200) ? 2200 : packet.data1;
                uint16_t safe_data2 = (packet.data2 < 800) ? 800 : (packet.data2 > 2200) ? 2200 : packet.data2;
                PWM_SetPulseWidth_CH3(safe_data1);
                PWM_SetPulseWidth_CH4(safe_data2);
            //}
        }
        // 无数据时保持当前PWM输出，不执行任何操作
    }
}
