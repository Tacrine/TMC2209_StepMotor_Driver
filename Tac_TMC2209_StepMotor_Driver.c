#include "Tac_TMC2209_StepMotor_Driver.h"
unsigned char SQW_Generator_En;  // 脉冲输出使能
uint32_t tick_count = 0;

// 设置电机方向
void set_Motor_Dir(Tac_StepMotor *motor_struct,unsigned char dir)
{
    if(dir == 'F')
    {
        #ifdef _MSPM0G3507_
            DL_GPIO_setPins(motor_struct->DirPort, motor_struct->DirPin);
        #endif
        #ifdef USE_HAL_DRIVER
            HAL_GPIO_WritePin(motor_struct->DirPort, motor_struct->DirPin, GPIO_PIN_SET);
        #endif
        motor_struct->Dir = 'F';
    }
    else if(dir == 'B')
    {
        #ifdef _MSPM0G3507_
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
        #ifdef _MSPM0G3507_
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
        #ifdef _MSPM0G3507_
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
        #ifdef _MSPM0G3507_
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
        #ifdef _MSPM0G3507_        
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
    #ifdef _MSPM0G3507_
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
    #ifdef _MSPM0G3507_
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






        #ifdef _MSPM0G3507_

        #endif
        #ifdef USE_HAL_DRIVER
        
        #endif







/// @brief 立即发生脉冲信号
/// @param motor_struct 所需要控制电机的配置结构体
/// @param freq 所需运动的频率(TMC2209按步驱动，一个脉冲电机就运动一步，频率越高，速度越快)
void enable_PWM(Tac_StepMotor *motor_struct, uint32_t freq)
{
    SQW_Generator_En = '1';
}

void SQW_Gen_Ticks(void)
{
    #ifdef _MSPM0G3507_
        #ifdef _TMC_GPIO_

        #endif
    #endif
    #ifdef USE_HAL_DRIVER
        #ifdef _TMC_GPIO_
            if(SQW_Generator_En == '1')
            {
                
            }
        #endif
    #endif
}

void SQW_Gen_Stop(void)
{
    SQW_Generator_En = '0';
}