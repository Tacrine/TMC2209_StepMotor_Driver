#include "Tac_TMC2209_StepMotor_Driver.h"

//脉冲发生器相关变量定义

#ifdef __MSPM0G3507__
void Tac_StepMotor_Init(Tac_StepMotor *motor_struct, DL_Tac_StepMotor_TimerConfig *timer_config,
                        GPIO_Regs *DirPort, uint32_t DirPin,
                        GPIO_Regs *MicrostepPort_A, uint32_t MicrostepPin_A,
                        GPIO_Regs *MicrostepPort_B, uint32_t MicrostepPin_B,
                        GPIO_Regs *LockPort, uint32_t LockPin,
                        GPIO_Regs *SQW_Port, uint32_t SQW_Pin)
{
    motor_struct->DirPort = DirPort;
    motor_struct->DirPin = DirPin;
    motor_struct->MicrostepPort_A = MicrostepPort_A;
    motor_struct->MicrostepPin_A = MicrostepPin_A;
    motor_struct->MicrostepPort_B = MicrostepPort_B;
    motor_struct->MicrostepPin_B = MicrostepPin_B;
    motor_struct->LockPort = LockPort;
    motor_struct->LockPin = LockPin;
    motor_struct->SQW_Port = SQW_Port;
    motor_struct->SQW_Pin = SQW_Pin;
    motor_struct->Lock = 'F';
    motor_struct->ticks = 0;
    motor_struct->SQW_tick_target = 0;
}
#endif
#ifdef USE_HAL_DRIVER
// 这里的GPIO_TypeDef * 是HAL库中的类型
void Tac_StepMotor_Init(Tac_StepMotor *motor_struct, TIM_HandleTypeDef *htim,
                        GPIO_TypeDef *dir_port, uint16_t dir_pin, 
                        GPIO_TypeDef *microstep_port_A, uint16_t microstep_pin_A, 
                        GPIO_TypeDef *microstep_port_B, uint16_t microstep_pin_B, 
                        GPIO_TypeDef *lock_port, uint16_t lock_pin, 
                        GPIO_TypeDef *SQW_port, 
                        uint16_t SQW_pin) 
{   
    motor_struct->htim = htim;  
    motor_struct->DirPort = dir_port;     
    motor_struct->DirPin = dir_pin;     
    motor_struct->MicrostepPort_A = microstep_port_A;     
    motor_struct->MicrostepPin_A = microstep_pin_A;     
    motor_struct->MicrostepPort_B = microstep_port_B;     
    motor_struct->MicrostepPin_B = microstep_pin_B;     
    motor_struct->LockPort = lock_port;     
    motor_struct->LockPin = lock_pin;     
    motor_struct->SQW_Port = SQW_port;     
    motor_struct->SQW_Pin = SQW_pin; 
    motor_struct->Lock = 'F';
    motor_struct->ticks = 0;
    motor_struct->SQW_tick_target = 0;
}
#endif


// 设置电机方向
void set_Motor_Dir(Tac_StepMotor *motor_struct,unsigned char dir)
{
    if(dir == 'F')
    {
        #ifdef __MSPM0G3507__
            DL_GPIO_setPins(motor_struct->DirPort, motor_struct->DirPin);
        #endif
        #ifdef USE_HAL_DRIVER
            HAL_GPIO_WritePin(motor_struct->DirPort, motor_struct->DirPin, GPIO_PIN_SET);
        #endif
        motor_struct->Dir = 'F';
    }
    else if(dir == 'B')
    {
        #ifdef __MSPM0G3507__
            DL_GPIO_clearPins(motor_struct->DirPort, motor_struct->DirPin);
        #endif
        #ifdef USE_HAL_DRIVER
            HAL_GPIO_WritePin(motor_struct->DirPort, motor_struct->DirPin, GPIO_PIN_RESET);
        #endif
        motor_struct->Dir = 'B';
    }
}

// 设置电机细分
void set_Microsteps(Tac_StepMotor *motor_struct, uint8_t microsteps)
{
    // 根据细分设置TMC2209的MSA和MSB引脚
    if (microsteps == 64)
    {
        #ifdef __MSPM0G3507__
            DL_GPIO_clearPins(motor_struct->MicrostepPort_A, motor_struct->MicrostepPin_A);
            DL_GPIO_setPins(motor_struct->MicrostepPort_B, motor_struct->MicrostepPin_B);
        #endif
        #ifdef USE_HAL_DRIVER
            HAL_GPIO_WritePin(motor_struct->MicrostepPort_A, motor_struct->MicrostepPin_A, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor_struct->MicrostepPort_B, motor_struct->MicrostepPin_B, GPIO_PIN_SET);
        #endif
        motor_struct->Microsteps = 64;
    }
    else if (microsteps == 32)
    {
        #ifdef __MSPM0G3507__
            DL_GPIO_setPins(motor_struct->MicrostepPort_A, motor_struct->MicrostepPin_A);
            DL_GPIO_clearPins(motor_struct->MicrostepPort_B, motor_struct->MicrostepPin_B);
        #endif
        #ifdef USE_HAL_DRIVER
            HAL_GPIO_WritePin(motor_struct->MicrostepPort_A, motor_struct->MicrostepPin_A, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor_struct->MicrostepPort_B, motor_struct->MicrostepPin_B, GPIO_PIN_RESET);
        #endif
        motor_struct->Microsteps = 32;
    }
    else if (microsteps == 16)
    {
        #ifdef __MSPM0G3507__
            DL_GPIO_setPins(motor_struct->MicrostepPort_A, motor_struct->MicrostepPin_A);
            DL_GPIO_setPins(motor_struct->MicrostepPort_B, motor_struct->MicrostepPin_B);
        #endif
        #ifdef USE_HAL_DRIVER
            HAL_GPIO_WritePin(motor_struct->MicrostepPort_A, motor_struct->MicrostepPin_A, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor_struct->MicrostepPort_B, motor_struct->MicrostepPin_B, GPIO_PIN_SET);
        #endif
        motor_struct->Microsteps = 16;
    }
    else if (microsteps == 8)
    {
        #ifdef __MSPM0G3507__        
            DL_GPIO_clearPins(motor_struct->MicrostepPort_A, motor_struct->MicrostepPin_A);
            DL_GPIO_clearPins(motor_struct->MicrostepPort_B, motor_struct->MicrostepPin_B);        
        #endif
        #ifdef USE_HAL_DRIVER
            HAL_GPIO_WritePin(motor_struct->MicrostepPort_A, motor_struct->MicrostepPin_A, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor_struct->MicrostepPort_B, motor_struct->MicrostepPin_B, GPIO_PIN_RESET);
        #endif
        motor_struct->Microsteps = 8;
    }
}

// 设置步进电机运行的步数
void set_Motor_Steps(Tac_StepMotor *motor_struct, uint16_t steps)
{
    motor_struct->Steps_remain = steps;
}

// 锁定步进电机
//TMC2209的锁定功能，长时间锁定会造成电机发热
void Motor_Lock(Tac_StepMotor *motor_struct)
{
    #ifdef __MSPM0G3507__
        DL_GPIO_clearPins(motor_struct->LockPort, motor_struct->LockPin);
    #endif
    #ifdef USE_HAL_DRIVER
        HAL_GPIO_WritePin(motor_struct->LockPort, motor_struct->LockPin, GPIO_PIN_RESET);
    #endif
    motor_struct->Lock = 'T';
}

// 解锁步进电机
void Motor_Unlock(Tac_StepMotor *motor_struct)
{
    #ifdef __MSPM0G3507__
        DL_GPIO_setPins(motor_struct->LockPort, motor_struct->LockPin);
    #endif
    #ifdef USE_HAL_DRIVER
        HAL_GPIO_WritePin(motor_struct->LockPort, motor_struct->LockPin, GPIO_PIN_SET);
    #endif
    motor_struct->Lock = 'F';
}

/// @brief 计算不同细分下步进电机转动自定义角度所需的步数
/// @param angle 需要转动的角度
/// @return 电机所需转动的步数
uint16_t Caculate_Steps(Tac_StepMotor *motor_struct, float angle)
{
    uint8_t microsteps_ = motor_struct->Microsteps;
    float steps = (200 * microsteps_ * angle) / 360.0;
    return (uint16_t)(steps + 0.5); // 四舍五入返回整数
}


/// @brief 电机正转自定义步数
/// @param motor_struct 所需要控制电机的配置结构体
/// @param steps 所需要运动的步数
/// @param microsteps 所需要的电机细分
/// @param lock_after_move 运动完毕后时候锁定，T为锁定，F为不锁定
/// @param freq 所需运动的频率(TMC2209按步驱动，一个脉冲电机就运动一步，频率越高，速度越快)
void Motor_Forward_Steps(Tac_StepMotor *motor_struct, uint16_t steps, uint8_t microsteps, unsigned char lock_after_move, uint32_t freq)
{
    // 设置电机方向
    set_Motor_Dir(motor_struct, 'F');
    // 设置电机细分
    set_Microsteps(motor_struct, microsteps);
    // 电机运动步数
    set_Motor_Steps(motor_struct, steps);
    // 解锁电机
    Motor_Unlock(motor_struct);
    // 启动脉冲信号,计数到脉冲数量足够后会自动停止
    enable_PWM(motor_struct, freq);
    if(lock_after_move == 'T')
    {
        Motor_Lock(motor_struct);
    }
    else
        Motor_Unlock(motor_struct);
}


/// @brief 电机正转自定义角度
/// @param motor_struct 所需要控制电机的配置结构体
/// @param angle 所需要运动的角度
/// @param microsteps 所需要的电机细分
/// @param lock_after_move 运动完毕后时候锁定，T为锁定，F为不锁定
/// @param freq 所需运动的频率(TMC2209按步驱动，一个脉冲电机就运动一步，频率越高，速度越快)
void Motor_Forward_Angle(Tac_StepMotor *motor_struct, float angle, uint8_t microsteps, unsigned char lock_after_move, uint32_t freq)
{
    // 设置电机方向
    set_Motor_Dir(motor_struct, 'F');
    // 设置电机细分
    set_Microsteps(motor_struct, microsteps);
    // 计算电机运动步数
    uint16_t steps = Caculate_Steps(motor_struct, angle);
    set_Motor_Steps(motor_struct, steps);
    // 解锁电机
    Motor_Unlock(motor_struct);
    // 启动脉冲信号,计数到脉冲数量足够后会自动停止
    enable_PWM(motor_struct, freq);
    if(lock_after_move == 'T')
    {
        Motor_Lock(motor_struct);
    }
    else
        Motor_Unlock(motor_struct);
}


/// @brief 电机反转自定义步数
/// @param motor_struct 所需要控制电机的配置结构体
/// @param steps 所需要运动的步数
/// @param microsteps 所需要的电机细分
/// @param lock_after_move 运动完毕后时候锁定，T为锁定，F为不锁定
/// @param freq 所需运动的频率(TMC2209按步驱动，一个脉冲电机就运动一步，频率越高，速度越快)
void Motor_Backward_Steps(Tac_StepMotor *motor_struct, uint16_t steps, uint8_t microsteps, unsigned char lock_after_move, uint32_t freq)
{
    // 设置电机方向
    set_Motor_Dir(motor_struct, 'B');
    // 设置电机细分
    set_Microsteps(motor_struct, microsteps);
    // 电机运动步数
    set_Motor_Steps(motor_struct, steps);
    // 解锁电机
    Motor_Unlock(motor_struct);
    // 启动脉冲信号,计数到脉冲数量足够后会自动停止
    enable_PWM(motor_struct, freq);
    if(lock_after_move == 'T')
    {
        Motor_Lock(motor_struct);
    }
    else
        Motor_Unlock(motor_struct);
}


/// @brief 电机反转自定义角度
/// @param motor_struct 所需要控制电机的配置结构体
/// @param angle 所需要运动的角度
/// @param microsteps 所需要的电机细分
/// @param lock_after_move 运动完毕后时候锁定，T为锁定，F为不锁定
/// @param freq 所需运动的频率(TMC2209按步驱动，一个脉冲电机就运动一步，频率越高，速度越快)
void Motor_Backward_Angle(Tac_StepMotor *motor_struct, float angle, uint8_t microsteps, unsigned char lock_after_move, uint32_t freq)
{
    // 设置电机方向
    set_Motor_Dir(motor_struct, 'B');
    // 设置电机细分
    set_Microsteps(motor_struct, microsteps);
    // 计算电机运动步数
    uint32_t steps = Caculate_Steps(motor_struct, angle);
    set_Motor_Steps(motor_struct, steps);
    // 解锁电机
    Motor_Unlock(motor_struct);
    // 启动脉冲信号,计数到脉冲数量足够后会自动停止
    enable_PWM(motor_struct, freq);
    if(lock_after_move == 'T')
    {
        Motor_Lock(motor_struct);
    }
    else
        Motor_Unlock(motor_struct);
}


/// @brief 立即发生脉冲信号
/// @param motor_struct 所需要控制电机的配置结构体
/// @param freq 所需运动的频率(TMC2209按步驱动，一个脉冲电机就运动一步，频率越高，速度越快)
void enable_PWM(Tac_StepMotor *motor_struct, uint32_t freq)
{
    motor_struct->ticks = 0;
    SQW_Set_Frequency(motor_struct, freq);
    motor_struct->SQW_Generator_En = '1';
    while(motor_struct->Steps_remain > 1)     // 循环到步数耗尽，多留一步防止数据溢出
    {}
    SQW_Gen_Stop(motor_struct);
}


/*以下内容为脉冲发生器部分*/


/// @brief 停止脉冲信号
/// @param motor_struct 所需要控制电机的配置结构体
void SQW_Gen_Stop(Tac_StepMotor *motor_struct)
{
    motor_struct->SQW_Generator_En = '0';
}


#ifdef __MSPM0G3507__
uint32_t Get_Config_Clock(Tac_StepMotor *motor_struct)
{
    uint32_t DL_TIMER_CLOCK_BUSCLK = motor_struct->timer_config->clockSel;
    return DL_TIMER_CLOCK_BUSCLK;
}

// 计算中断频率
float Get_TIM_Update_Freq(Tac_StepMotor *motor_struct) 
{
    //DL_TimerA_getClockConfig(gptimer, config);  
    //uint32_t DL_Timer_LoadValue = DL_TimerA_getLoadValue(gptimer);
    uint32_t period = motor_struct->timer_config->period;
    return period;
}


/// @brief 根据目标频率自动计算所需要的tick数量
/// @param htim 定时器句柄
/// @param motor_struct 所需要控制电机的配置结构体 
/// @param freq_out 输出的频率
void SQW_Set_Frequency(Tac_StepMotor *motor_struct , float freq_out)
{
    float f_int = Get_TIM_Update_Freq(motor_struct);

    motor_struct->SQW_tick_target = (uint32_t)(freq_out / f_int);

    if (motor_struct->SQW_tick_target < 1)
        motor_struct->SQW_tick_target = 1;   // 防止除零
        
}

#endif



#ifdef USE_HAL_DRIVER
/// @brief 获取当前定时器APB值，用以计算周期等
/// @param htim 定时器句柄
/// @return 定时器周期
uint32_t Get_TIM_Clock(TIM_HandleTypeDef *htim)
{
    uint32_t pclk, ppre, timclk;

    // 判断定时器属于 APB2 还是 APB1 
    if (htim->Instance == TIM1 )  /* || htim->Instance == TIM8  ||
        htim->Instance == TIM15 || htim->Instance == TIM16 ||                   //这里报错的话则需要手动确认定时器
        htim->Instance == TIM17) */
    {
        /* APB2 定时器 */
        pclk = HAL_RCC_GetPCLK2Freq();
        ppre = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
    }
    else
    {
        // APB1 定时器 
        pclk = HAL_RCC_GetPCLK1Freq();
        ppre = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
    }

    // APB 预分频器 > 1 时，定时器时钟加倍 
    if (ppre >= 4)   // 100b=div2, 101b=div4, 110b=div8, 111b=div16
        timclk = pclk * 2;
    else
        timclk = pclk;

    return timclk;
}


//自动计算定时器中断频率
float Get_TIM_Update_Freq(TIM_HandleTypeDef *htim)
{
    uint32_t timclk = Get_TIM_Clock(htim);
    uint32_t psc = htim->Instance->PSC;
    uint32_t arr = htim->Instance->ARR;

    return (float)timclk / ((psc + 1) * (arr + 1));
}


/// @brief 根据目标频率自动计算所需要的tick数量
/// @param htim 定时器句柄
/// @param motor_struct 所需要控制电机的配置结构体 
/// @param freq_out 输出的频率
void SQW_Set_Frequency(Tac_StepMotor *motor_struct , float freq_out)
{
    float f_int = Get_TIM_Update_Freq(motor_struct->htim);

    motor_struct->SQW_tick_target = (uint32_t)(freq_out / f_int);

    if (motor_struct->SQW_tick_target < 1)
        motor_struct->SQW_tick_target = 1;   // 防止除零
}


/// @brief 计算方波输出的函数，需要将该函数放入中断回调中去
/// @param motor_struct 所需要控制电机的配置结构体
void SQW_Gen(Tac_StepMotor *motor_struct)
{
    if(motor_struct->SQW_Generator_En == '1')
    {
        motor_struct->ticks++;
        if (motor_struct->ticks >= motor_struct->SQW_tick_target)
        {
            HAL_GPIO_WritePin(motor_struct->SQW_Port, motor_struct->SQW_Pin, GPIO_PIN_SET); // 输出方波
            motor_struct->Steps_remain--;
            motor_struct->ticks = 0;
        }
        else if (motor_struct->Steps_remain <= ((uint8_t)1))  //留出多余步数，防止溢出，原本应该为0
        {
            SQW_Gen_Stop(motor_struct);
        }
        else
            HAL_GPIO_WritePin(motor_struct->SQW_Port, motor_struct->SQW_Pin,GPIO_PIN_RESET);
    }
}
#endif
