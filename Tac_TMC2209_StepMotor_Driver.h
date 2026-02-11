// 作者：Tacrine_F
// 时间：26年2月
// 版本：V0.1alpha 只通过编译，等待实际测试
/*
说明：
    跨平台：本驱动程序使用TMC2209驱动芯片以驱动42步进电机,支持跨平台调用，STM32 HAL和MSP430 G3507平台。
    使用需要设置编译器预定义宏:
    一般来说工程生成完毕时，会有预定义宏，如果没有，请手动添加
        __MSPM0G3507__ 或者 USE_HAL_DRIVER
    脉冲发生：本驱动有多个实现脉冲的方式，用户可以根据自己的需求选择。目前只支持_TMC_GPIO_模式，_TMC_TIMER_模式还未实现。
    使用需要设置编译器预定义宏: _TMC_GPIO_ 或者 _TMC_TIMER_
        _TMC_GPIO_模式：使用定时器中断来翻转引脚电平，需要在中断函数中调用，只适合与约1KHz左右的速度，高速会频繁进入中断，打乱MCU运行周期。
        下文是示例。
                    ST_HAL：
                        extern Tac_StepMotor Step_Motor_X;                      // 先引入步进电机结构体
                        extern Tac_StepMotor Step_Motor_Y;
                        HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
                        {
                            if (htim == (&htim6))                               // 选择使用的定时器
                            {
                                SQW_Gen(&motor_struct_X);
                                SQW_Gen(&motor_struct_Y);
                            }
                        }
                    TIMSPM0G3507:
                        extern Tac_StepMotor Step_Motor_X;                      // 先引入步进电机结构体
                        extern Tac_StepMotor Step_Motor_Y;
                        void TIMER_0_INST_IRQHandler(void)                      // 选择使用的定时器
                        {
                            switch (DL_TimerA_getPendingInterrupt(TIMER_0_INST))
                            {
                                case DL_TIMERA_IIDX_REPEAT_COUNT:
                                    SQW_Gen(&motor_struct_X);
                                    SQW_Gen(&motor_struct_Y);
                                    break;
                                default:                                    // 其他中断不处理
                                    break;
                            }
                        }

        _TMC_TIMER_模式：直接使用定时器来产生脉冲，STM32需要配置主从定时器。
*/

#ifndef _Tac_TMC2209_StepMotor_Driver_h_
#define _Tac_TMC2209_StepMotor_Driver_h_

#include <stdint.h>
#include <stdbool.h>


// 引入TI G3507的头文件
#ifdef __MSPM0G3507__
#include "ti/driverlib/dl_gpio.h"
#include "ti/driverlib/m0p/dl_interrupt.h"
#include "ti_msp_dl_config.h"

// TI G3507的时钟以及定时器配置结构体
typedef struct
{
    DL_TIMER_CLOCK clockSel;
    DL_TIMER_CLOCK_DIVIDE divideRatio;
    uint8_t prescale;
    uint32_t period;
} DL_Tac_StepMotor_TimerConfig;

// 步进电机参数定义
typedef struct
{
    // 步进电机引脚定义
    uint32_t DirPin;
    GPIO_Regs *DirPort;
    uint32_t StepPin;
    GPIO_Regs *StepPort;
    uint32_t LockPin;
    GPIO_Regs *LockPort;
    uint32_t MicrostepPin_A;
    GPIO_Regs *MicrostepPort_A;
    uint32_t MicrostepPin_B;
    GPIO_Regs *MicrostepPort_B;
    uint32_t SQW_Pin;
    GPIO_Regs *SQW_Port;

    // 步进电机运动状态设置
    uint32_t Steps_remain; // 电机需要运动的步数
    uint8_t Microsteps;    // 细分
    unsigned char Dir;     // 方向,T为顺时针，F为逆时针(从电机运动轴的上面看)
    unsigned char Lock;    // 锁定状态，T为锁定，F为未锁定

    DL_Tac_StepMotor_TimerConfig *timer_config; // 定时器配置结构体
    uint16_t Freq;                              // 脉冲频率(Hz)
    unsigned char SQW_Generator_En;             // 脉冲输出使能
    uint32_t ticks;                             // 定时器时刻，用以计算脉冲周期
    uint32_t SQW_tick_target;                   // 脉冲周期目标值
} Tac_StepMotor;

// 初始化函数
void Tac_StepMotor_Init(Tac_StepMotor *motor_struct, DL_Tac_StepMotor_TimerConfig *timer_config,
                        GPIO_Regs *DirPort, uint32_t DirPin,
                        GPIO_Regs *MicrostepPort_A, uint32_t MicrostepPin_A,
                        GPIO_Regs *MicrostepPort_B, uint32_t MicrostepPin_B,
                        GPIO_Regs *LockPort, uint32_t LockPin,
                        GPIO_Regs *SQW_Port, uint32_t SQW_Pin);
#endif

// 引入STM32 HAL的头文件
#ifdef USE_HAL_DRIVER
#include "main.h"

// 步进电机参数定义
typedef struct
{
    // 步进电机引脚定义
    uint16_t DirPin;
    GPIO_TypeDef *DirPort;
    uint16_t StepPin;
    GPIO_TypeDef *StepPort;
    uint16_t LockPin;
    GPIO_TypeDef *LockPort;
    uint16_t MicrostepPin_A;
    GPIO_TypeDef *MicrostepPort_A;
    uint16_t MicrostepPin_B;
    GPIO_TypeDef *MicrostepPort_B;
    uint16_t SQW_Pin;
    GPIO_TypeDef *SQW_Port;

    // 步进电机运动状态设置
    uint32_t Steps_remain; // 电机需要运动的步数
    uint8_t Microsteps;    // 细分
    unsigned char Dir;     // 方向,T为顺时针，F为逆时针(从电机运动轴的上面看)
    unsigned char Lock;    // 锁定状态，T为锁定，F为未锁定

    TIM_HandleTypeDef *htim;        // 定时器句柄
    uint16_t Freq;                  // 脉冲频率(Hz)
    unsigned char SQW_Generator_En; // 脉冲输出使能
    uint32_t ticks;                 // 定时器时刻，用以计算脉冲周期
    uint32_t SQW_tick_target;       // 脉冲周期目标值

} Tac_StepMotor;

// 初始化函数
void Tac_StepMotor_Init(Tac_StepMotor *motor_struct, TIM_HandleTypeDef *htim,
                        GPIO_TypeDef *dir_port, uint16_t dir_pin,
                        GPIO_TypeDef *microstep_port_A, uint16_t microstep_pin_A,
                        GPIO_TypeDef *microstep_port_B, uint16_t microstep_pin_B,
                        GPIO_TypeDef *lock_port, uint16_t lock_pin,
                        GPIO_TypeDef *SQW_port, uint16_t SQW_pin);

#endif

void set_Motor_Dir(Tac_StepMotor *motor_struct, unsigned char dir);
void set_Microsteps(Tac_StepMotor *motor_struct, uint8_t microsteps);
void set_Motor_Steps(Tac_StepMotor *motor_struct, uint16_t steps);

void Motor_Lock(Tac_StepMotor *motor_struct);
void Motor_Unlock(Tac_StepMotor *motor_struct);

uint16_t Caculate_Steps(Tac_StepMotor *motor_struct, float angle);

void Motor_Forward_Steps(Tac_StepMotor *motor_struct, uint16_t steps, uint8_t microsteps, unsigned char lock_after_move, uint32_t freq);
void Motor_Backward_Steps(Tac_StepMotor *motor_struct, uint16_t steps, uint8_t microsteps, unsigned char lock_after_move, uint32_t freq);

void Motor_Forward_Angle(Tac_StepMotor *motor_struct, float angle, uint8_t microsteps, unsigned char lock_after_move, uint32_t freq);
void Motor_Backward_Angle(Tac_StepMotor *motor_struct, float angle, uint8_t microsteps, unsigned char lock_after_move, uint32_t freq);

void enable_PWM(Tac_StepMotor *motor_struct, uint32_t freq);
void SQW_Gen(Tac_StepMotor *motor_struct);
void SQW_Gen_Stop(Tac_StepMotor *motor_struct);

void SQW_Set_Frequency(Tac_StepMotor *motor_struct, float freq_out);    //HAL库获取定时器时钟频率，并设置脉冲频率

#endif // _Tac_TMC2209_StepMotor_Driver_h_